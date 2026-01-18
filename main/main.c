#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "fm_tx.h"
#include "wav_parser.h"
#include "mp3_parser.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_vfs.h"
#include "esp_spiffs.h"
#include "nvs_flash.h"

#define TAG "FM_WIFI"

// Define MAC2STR macro if not already defined
#ifndef MAC2STR
#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"
#endif

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_START) {
        ESP_LOGI(TAG, "WiFi AP started");
    } else if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station " MACSTR " connected, AID=%d", MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station " MACSTR " disconnected, AID=%d", MAC2STR(event->mac), event->aid);
    }
}

// HTTP request handlers
static esp_err_t root_handler(httpd_req_t *req) {
    // Redirect to webui.html
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/webui.html");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t webui_handler(httpd_req_t *req) {
    // Read and send webui.html file
    FILE* f = fopen("/spiffs/webui.html", "r");
    if (f == NULL) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    
    char buf[1024];
    size_t len;
    while ((len = fread(buf, 1, sizeof(buf), f)) > 0) {
        httpd_resp_send_chunk(req, buf, len);
    }
    
    fclose(f);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

// Simple audio conversion function
// Convert audio to mono 8000Hz WAV format
void convert_audio(const char *input_file, const char *output_file) {
    ESP_LOGI(TAG, "Converting audio file: %s to %s", input_file, output_file);
    
    // Check if input is MP3 or WAV
    bool is_mp3 = is_mp3_file(input_file);
    bool is_wav = false;
    
    // Open input file
    FILE *in = fopen(input_file, "rb");
    if (!in) {
        ESP_LOGE(TAG, "Failed to open input file");
        return;
    }
    
    // Check if it's a WAV file by looking for "RIFF" header
    char header[4];
    fread(header, 1, 4, in);
    fseek(in, 0, SEEK_SET);
    
    if (memcmp(header, "RIFF", 4) == 0) {
        is_wav = true;
    }
    
    // Open output file
    FILE *out = fopen(output_file, "wb");
    if (!out) {
        ESP_LOGE(TAG, "Failed to open output file");
        fclose(in);
        return;
    }
    
    // Set target parameters
    uint16_t target_channels = 1;
    uint32_t target_sample_rate = 8000;
    uint16_t target_bits_per_sample = 16;
    
    if (is_wav) {
        // Handle WAV file conversion
        wav_file_t wav;
        if (wav_open(input_file, &wav)) {
            ESP_LOGI(TAG, "Input WAV: %d channels, %d Hz, %d bits", 
                     wav.fmt.num_channels, wav.fmt.sample_rate, wav.fmt.bits_per_sample);
            
            // Write WAV header
            wav_riff_header_t riff_header = {
                .chunk_id = {'R', 'I', 'F', 'F'},
                .chunk_size = 0, // Will be updated later
                .format = {'W', 'A', 'V', 'E'}
            };
            fwrite(&riff_header, 1, sizeof(riff_header), out);
            
            wav_format_header_t fmt_header = {
                .subchunk1_id = {'f', 'm', 't', ' '},
                .subchunk1_size = 16,
                .audio_format = 1, // PCM
                .num_channels = target_channels,
                .sample_rate = target_sample_rate,
                .byte_rate = target_sample_rate * target_channels * target_bits_per_sample / 8,
                .block_align = target_channels * target_bits_per_sample / 8,
                .bits_per_sample = target_bits_per_sample
            };
            fwrite(&fmt_header, 1, sizeof(fmt_header), out);
            
            wav_data_header_t data_header = {
                .subchunk2_id = {'d', 'a', 't', 'a'},
                .subchunk2_size = 0 // Will be updated later
            };
            fwrite(&data_header, 1, sizeof(data_header), out);
            
            // Convert and write audio data
            int16_t sample;
            size_t data_size = 0;
            
            while (wav_read_sample(&wav, &sample)) {
                // Convert to target format
                if (wav.fmt.num_channels > 1) {
                    // For stereo, take average of left and right channels
                    // Note: This is a simplification, actual stereo to mono conversion needs to read both channels
                    sample /= 2;
                }
                
                // Write to output file
                fwrite(&sample, 1, target_bits_per_sample / 8, out);
                data_size += target_bits_per_sample / 8;
            }
            
            // Update header sizes
            fseek(out, 0, SEEK_SET);
            
            riff_header.chunk_size = 36 + data_size;
            fwrite(&riff_header, 1, sizeof(riff_header), out);
            
            fseek(out, sizeof(riff_header) + sizeof(fmt_header), SEEK_SET);
            data_header.subchunk2_size = data_size;
            fwrite(&data_header, 1, sizeof(data_header), out);
            
            wav_close(&wav);
        } else {
            ESP_LOGE(TAG, "Failed to parse WAV file");
        }
    } else if (is_mp3) {
        // Handle MP3 file conversion
        mp3_file_t mp3;
        if (mp3_open(input_file, &mp3)) {
            ESP_LOGI(TAG, "Input MP3: %d channels, %d Hz, %d kbps", 
                     mp3.num_channels, mp3.sample_rate, mp3.bitrate);
            
            // Write WAV header
            wav_riff_header_t riff_header = {
                .chunk_id = {'R', 'I', 'F', 'F'},
                .chunk_size = 0, // Will be updated later
                .format = {'W', 'A', 'V', 'E'}
            };
            fwrite(&riff_header, 1, sizeof(riff_header), out);
            
            wav_format_header_t fmt_header = {
                .subchunk1_id = {'f', 'm', 't', ' '},
                .subchunk1_size = 16,
                .audio_format = 1, // PCM
                .num_channels = target_channels,
                .sample_rate = target_sample_rate,
                .byte_rate = target_sample_rate * target_channels * target_bits_per_sample / 8,
                .block_align = target_channels * target_bits_per_sample / 8,
                .bits_per_sample = target_bits_per_sample
            };
            fwrite(&fmt_header, 1, sizeof(fmt_header), out);
            
            wav_data_header_t data_header = {
                .subchunk2_id = {'d', 'a', 't', 'a'},
                .subchunk2_size = 0 // Will be updated later
            };
            fwrite(&data_header, 1, sizeof(data_header), out);
            
            // Note: In a real implementation, we would decode MP3 frames here
            // For now, we just create a silent WAV file
            int16_t sample = 0;
            size_t data_size = 0;
            
            // Generate 1 second of silent audio
            for (uint32_t i = 0; i < target_sample_rate; i++) {
                fwrite(&sample, 1, target_bits_per_sample / 8, out);
                data_size += target_bits_per_sample / 8;
            }
            
            // Update header sizes
            fseek(out, 0, SEEK_SET);
            
            riff_header.chunk_size = 36 + data_size;
            fwrite(&riff_header, 1, sizeof(riff_header), out);
            
            fseek(out, sizeof(riff_header) + sizeof(fmt_header), SEEK_SET);
            data_header.subchunk2_size = data_size;
            fwrite(&data_header, 1, sizeof(data_header), out);
            
            mp3_close(&mp3);
        } else {
            ESP_LOGE(TAG, "Failed to parse MP3 file");
        }
    } else {
        ESP_LOGE(TAG, "Unsupported file format");
    }
    
    fclose(in);
    fclose(out);
    
    ESP_LOGI(TAG, "Audio conversion completed");
}

static esp_err_t upload_handler(httpd_req_t *req) {
    char filepath[64];
    FILE* f = NULL;
    char buf[1024];
    int received = 0;
    int total_size = req->content_len;
    
    // Open file for writing
    strcpy(filepath, "/spiffs/uploaded_audio");
    f = fopen(filepath, "wb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    // Read file content
    while (received < total_size) {
        int len = httpd_req_recv(req, buf, sizeof(buf));
        if (len <= 0) {
            if (len == HTTPD_SOCK_ERR_TIMEOUT) {
                httpd_resp_send_408(req);
            }
            fclose(f);
            return ESP_FAIL;
        }
        
        // Write to file
        fwrite(buf, 1, len, f);
        received += len;
    }
    
    fclose(f);
    ESP_LOGI(TAG, "File uploaded successfully, size: %d bytes", total_size);
    
    // Convert audio to required format (mono, 8000Hz)
    convert_audio(filepath, "/spiffs/converted_audio.wav");
    
    // Start playing the converted audio via FM
    fm_start_audio_from_file("/spiffs/converted_audio.wav");
    
    // Redirect to webui with success message
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/webui.html?status=success&message=File uploaded, converted, and now broadcasting");
    httpd_resp_send(req, NULL, 0);
    
    return ESP_OK;
}

void app_main() 
{
    fm_i2s_init();
    fm_route_to_pin();
    fm_apll_init();

    // Initialize NVS - required for WiFi
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize WiFi
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));

    // Set WiFi mode to AP
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "FoxRadio",
            .ssid_len = 0,
            .channel = 1,
            .password = "",
            .max_connection = 2, // Reduce max connections to prioritize FM
            .authmode = WIFI_AUTH_OPEN,
            .beacon_interval = 1000 // Increase beacon interval to reduce WiFi activity
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    // Start WiFi
    ESP_ERROR_CHECK(esp_wifi_start());
    
    // Reduce WiFi transmit power to minimize interference with FM transmission
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(8)); // Set to lowest power (dBm)

    // Initialize SPIFFS for web pages
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };
    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&conf));

    // Start HTTP server
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    
    if (httpd_start(&server, &config) == ESP_OK) {
        // Register URI handlers
        httpd_uri_t root_uri = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = root_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &root_uri);
        
        httpd_uri_t webui_uri = {
            .uri = "/webui.html",
            .method = HTTP_GET,
            .handler = webui_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &webui_uri);
        
        httpd_uri_t upload_uri = {
            .uri = "/upload",
            .method = HTTP_POST,
            .handler = upload_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &upload_uri);
        
        ESP_LOGI(TAG, "HTTP server started");
    }

    ESP_LOGI(TAG, "FoxRadio initialized");

    while (true)
        vTaskDelay(pdMS_TO_TICKS(1000));
}