#include "mp3_parser.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

// Simple MP3 frame header structure
#pragma pack(push, 1)
typedef struct {
    uint8_t sync[3];          // Sync bits (11 bits)
    uint8_t version_layer;    // Version and layer (4 bits)
    uint8_t protection_bit;   // Protection bit (1 bit)
    uint8_t bitrate_index;    // Bitrate index (4 bits)
    uint8_t sample_rate_index;// Sample rate index (2 bits)
    uint8_t padding_bit;      // Padding bit (1 bit)
    uint8_t private_bit;      // Private bit (1 bit)
    uint8_t channel_mode;     // Channel mode (2 bits)
    uint8_t mode_extension;   // Mode extension (2 bits)
    uint8_t copyright;        // Copyright (1 bit)
    uint8_t original;         // Original (1 bit)
    uint8_t emphasis;         // Emphasis (2 bits)
} mp3_frame_header_t;
#pragma pack(pop)

// Bitrate values for different versions and layers
static const uint16_t bitrate_table[4][3][15] = {
    // MPEG Version 1
    {
        {0, 32, 64, 96, 128, 160, 192, 224, 256, 288, 320, 352, 384, 416, 448}, // Layer I
        {0, 32, 48, 56,  64,  80,  96, 112, 128, 160, 192, 224, 256, 320, 384}, // Layer II
        {0, 32, 40, 48,  56,  64,  80,  96, 112, 128, 160, 192, 224, 256, 320}  // Layer III
    },
    // MPEG Version 2
    {
        {0, 32, 48, 56,  64,  80,  96, 112, 128, 144, 160, 176, 192, 224, 256}, // Layer I
        {0,  8, 16, 24,  32,  40,  48,  56,  64,  80,  96, 112, 128, 144, 160}, // Layer II
        {0,  8, 16, 24,  32,  40,  48,  56,  64,  80,  96, 112, 128, 144, 160}  // Layer III
    },
    // MPEG Version 2.5
    {
        {0, 32, 48, 56,  64,  80,  96, 112, 128, 144, 160, 176, 192, 224, 256}, // Layer I
        {0,  8, 16, 24,  32,  40,  48,  56,  64,  80,  96, 112, 128, 144, 160}, // Layer II
        {0,  8, 16, 24,  32,  40,  48,  56,  64,  80,  96, 112, 128, 144, 160}  // Layer III
    },
    // Invalid version
    {
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
    }
};

// Sample rate values for different versions
static const uint32_t sample_rate_table[4][3] = {
    {44100, 48000, 32000}, // MPEG Version 1
    {22050, 24000, 16000}, // MPEG Version 2
    {11025, 12000,  8000}, // MPEG Version 2.5
    {0,     0,     0}      // Invalid version
};

// Check if data starts with ID3v2 tag
bool has_id3v2_tag(const uint8_t *data, uint32_t size) {
    if (size < 10) return false;
    return (memcmp(data, "ID3", 3) == 0);
}

// Check if data ends with ID3v1 tag
bool has_id3v1_tag(const uint8_t *data, uint32_t size) {
    if (size < 128) return false;
    return (memcmp(data + size - 128, "TAG", 3) == 0);
}

// Calculate ID3v2 tag size
uint32_t get_id3v2_size(const uint8_t *data) {
    if (memcmp(data, "ID3", 3) != 0) return 0;
    return (((data[6] & 0x7F) << 21) | 
            ((data[7] & 0x7F) << 14) | 
            ((data[8] & 0x7F) << 7) | 
            (data[9] & 0x7F)) + 10; // Add header size
}

// Find first MP3 frame in data
uint32_t find_first_mp3_frame(const uint8_t *data, uint32_t size, mp3_frame_header_t *header) {
    for (uint32_t i = 0; i < size - 4; i++) {
        // Check for sync word (11 bits)
        if (data[i] == 0xFF && (data[i+1] & 0xE0) == 0xE0) {
            // Copy frame header
            memcpy(header, &data[i], 4);
            
            // Validate frame header
            uint8_t version = (header->version_layer >> 3) & 0x03;
            uint8_t layer = (header->version_layer >> 1) & 0x03;
            uint8_t bitrate_idx = header->bitrate_index & 0x0F;
            uint8_t sample_rate_idx = (header->sample_rate_index >> 2) & 0x03;
            
            // Check for valid values
            if (version == 1 || version == 2 || version == 0) {
                if (layer == 3 || layer == 2 || layer == 1) {
                    if (bitrate_idx != 0x0F && bitrate_idx != 0x00) {
                        if (sample_rate_idx != 0x03) {
                            return i;
                        }
                    }
                }
            }
        }
    }
    return 0;
}

bool mp3_open(const char *filename, mp3_file_t *mp3) {
    assert(filename && mp3);
    
    // Clear the MP3 structure
    memset(mp3, 0, sizeof(mp3_file_t));
    
    // Open the MP3 file
    mp3->file = fopen(filename, "rb");
    if (!mp3->file) {
        return false;
    }
    
    // Get file size
    fseek(mp3->file, 0, SEEK_END);
    uint32_t file_size = ftell(mp3->file);
    fseek(mp3->file, 0, SEEK_SET);
    
    // Check for ID3v2 tag
    uint8_t header[10];
    fread(header, 1, sizeof(header), mp3->file);
    fseek(mp3->file, 0, SEEK_SET);
    
    if (has_id3v2_tag(header, file_size)) {
        mp3->has_id3v2 = true;
        mp3->id3v2_size = get_id3v2_size(header);
        fseek(mp3->file, mp3->id3v2_size, SEEK_SET);
    }
    
    // Find first MP3 frame
    mp3_frame_header_t frame_header;
    uint32_t frame_pos = 0;
    
    // Read some data to find frame
    uint8_t buffer[1024];
    uint32_t bytes_read = fread(buffer, 1, sizeof(buffer), mp3->file);
    fseek(mp3->file, mp3->id3v2_size, SEEK_SET);
    
    if (bytes_read > 0) {
        frame_pos = find_first_mp3_frame(buffer, bytes_read, &frame_header);
        if (frame_pos > 0) {
            // Extract audio information
            uint8_t version = (frame_header.version_layer >> 3) & 0x03;
            uint8_t layer = (frame_header.version_layer >> 1) & 0x03;
            uint8_t bitrate_idx = frame_header.bitrate_index & 0x0F;
            uint8_t sample_rate_idx = (frame_header.sample_rate_index >> 2) & 0x03;
            uint8_t channel_mode = (frame_header.channel_mode >> 6) & 0x03;
            
            // Calculate values
            mp3->sample_rate = sample_rate_table[version][sample_rate_idx];
            mp3->bitrate = bitrate_table[version][3 - layer][bitrate_idx];
            mp3->num_channels = (channel_mode == 3) ? 1 : 2;
            
            // Set data position
            mp3->data_pos = mp3->id3v2_size + frame_pos;
            fseek(mp3->file, mp3->data_pos, SEEK_SET);
        }
    }
    
    // Check for ID3v1 tag
    if (file_size >= 128) {
        fseek(mp3->file, file_size - 128, SEEK_SET);
        fread(header, 1, 3, mp3->file);
        if (memcmp(header, "TAG", 3) == 0) {
            mp3->has_id3v1 = true;
        }
        fseek(mp3->file, mp3->data_pos, SEEK_SET);
    }
    
    mp3->is_open = true;
    mp3->is_memory_mode = false;
    
    return true;
}

bool mp3_open_from_memory(const uint8_t *data, uint32_t size, mp3_file_t *mp3) {
    assert(data && mp3);
    
    // Clear the MP3 structure
    memset(mp3, 0, sizeof(mp3_file_t));
    
    // Check for ID3v2 tag
    if (has_id3v2_tag(data, size)) {
        mp3->has_id3v2 = true;
        mp3->id3v2_size = get_id3v2_size(data);
    }
    
    // Find first MP3 frame
    mp3_frame_header_t frame_header;
    uint32_t frame_pos = find_first_mp3_frame(data + mp3->id3v2_size, size - mp3->id3v2_size, &frame_header);
    
    if (frame_pos > 0) {
        // Extract audio information
        uint8_t version = (frame_header.version_layer >> 3) & 0x03;
        uint8_t layer = (frame_header.version_layer >> 1) & 0x03;
        uint8_t bitrate_idx = frame_header.bitrate_index & 0x0F;
        uint8_t sample_rate_idx = (frame_header.sample_rate_index >> 2) & 0x03;
        uint8_t channel_mode = (frame_header.channel_mode >> 6) & 0x03;
        
        // Calculate values
        mp3->sample_rate = sample_rate_table[version][sample_rate_idx];
        mp3->bitrate = bitrate_table[version][3 - layer][bitrate_idx];
        mp3->num_channels = (channel_mode == 3) ? 1 : 2;
        
        // Set data position
        mp3->data_pos = mp3->id3v2_size + frame_pos;
    }
    
    // Check for ID3v1 tag
    if (has_id3v1_tag(data, size)) {
        mp3->has_id3v1 = true;
    }
    
    mp3->memory = data;
    mp3->memory_size = size;
    mp3->is_open = true;
    mp3->is_memory_mode = true;
    
    return true;
}

bool mp3_read_sample(mp3_file_t *mp3, int16_t *sample) {
    assert(mp3 && sample);
    
    if (!mp3->is_open) {
        return false;
    }
    
    // For this simple implementation, we just return 0
    // In a real implementation, we would decode MP3 frames
    *sample = 0;
    return true;
}

void mp3_close(mp3_file_t *mp3) {
    assert(mp3);
    
    if (mp3->is_open && mp3->file) {
        fclose(mp3->file);
        mp3->is_open = false;
        mp3->file = NULL;
    }
}

void mp3_reset(mp3_file_t *mp3) {
    assert(mp3);
    
    if (mp3->is_open) {
        if (mp3->is_memory_mode) {
            // In memory mode, just reset the data position
            mp3->data_pos = mp3->id3v2_size;
        } else if (mp3->file) {
            // In file mode, reset the file pointer
            fseek(mp3->file, mp3->data_pos, SEEK_SET);
        }
    }
}

bool is_mp3_file(const char *filename) {
    FILE *file = fopen(filename, "rb");
    if (!file) return false;
    
    uint8_t buffer[10];
    size_t bytes_read = fread(buffer, 1, sizeof(buffer), file);
    fclose(file);
    
    if (bytes_read < 10) return false;
    
    // Check for ID3v2 tag
    if (has_id3v2_tag(buffer, bytes_read)) return true;
    
    // Check for MP3 frame
    mp3_frame_header_t header;
    return (find_first_mp3_frame(buffer, bytes_read, &header) > 0);
}

bool is_mp3_data(const uint8_t *data, uint32_t size) {
    if (size < 4) return false;
    
    // Check for ID3v2 tag
    if (has_id3v2_tag(data, size)) return true;
    
    // Check for MP3 frame
    mp3_frame_header_t header;
    return (find_first_mp3_frame(data, size, &header) > 0);
}