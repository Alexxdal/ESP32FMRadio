#include "wav_parser.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

bool wav_open(const char *filename, wav_file_t *wav) {
    assert(filename && wav);
    
    // Clear the WAV structure
    memset(wav, 0, sizeof(wav_file_t));
    
    // Open the WAV file
    wav->file = fopen(filename, "rb");
    if (!wav->file) {
        return false;
    }
    
    // Read RIFF header
    if (fread(&wav->riff, sizeof(wav_riff_header_t), 1, wav->file) != 1) {
        fclose(wav->file);
        return false;
    }
    
    // Check RIFF signature
    if (memcmp(wav->riff.chunk_id, "RIFF", 4) != 0 || 
        memcmp(wav->riff.format, "WAVE", 4) != 0) {
        fclose(wav->file);
        return false;
    }
    
    // Read format header
    if (fread(&wav->fmt, sizeof(wav_format_header_t), 1, wav->file) != 1) {
        fclose(wav->file);
        return false;
    }
    
    // Check format signature
    if (memcmp(wav->fmt.subchunk1_id, "fmt ", 4) != 0) {
        fclose(wav->file);
        return false;
    }
    
    // Check if it's PCM format
    if (wav->fmt.audio_format != 1) {
        fclose(wav->file);
        return false;
    }
    
    // Check sample format (8 or 16 bits)
    if (wav->fmt.bits_per_sample != 8 && wav->fmt.bits_per_sample != 16) {
        fclose(wav->file);
        return false;
    }
    
    // Find data chunk
    wav_data_header_t data_header;
    bool found_data = false;
    
    while (!found_data) {
        // Read subchunk header
        if (fread(&data_header, sizeof(data_header.subchunk2_id) + sizeof(data_header.subchunk2_size), 1, wav->file) != 1) {
            fclose(wav->file);
            return false;
        }
        
        // Check if it's data chunk
        if (memcmp(data_header.subchunk2_id, "data", 4) == 0) {
            found_data = true;
            wav->data = data_header;
            break;
        } else {
            // Skip other chunks
            uint32_t chunk_size = data_header.subchunk2_size;
            if (fseek(wav->file, chunk_size, SEEK_CUR) != 0) {
                fclose(wav->file);
                return false;
            }
        }
    }
    
    if (!found_data) {
        fclose(wav->file);
        return false;
    }
    
    // Record data offset
    wav->data_offset = ftell(wav->file);
    wav->data_pos = 0;
    wav->is_open = true;
    wav->is_memory_mode = false;
    
    return true;
}

bool wav_open_from_memory(const uint8_t *data, uint32_t size, wav_file_t *wav) {
    assert(data && wav);
    
    // Clear the WAV structure
    memset(wav, 0, sizeof(wav_file_t));
    
    // Check if buffer is large enough for RIFF header
    if (size < sizeof(wav_riff_header_t)) {
        return false;
    }
    
    // Copy RIFF header
    memcpy(&wav->riff, data, sizeof(wav_riff_header_t));
    
    // Check RIFF signature
    if (memcmp(wav->riff.chunk_id, "RIFF", 4) != 0 || 
        memcmp(wav->riff.format, "WAVE", 4) != 0) {
        return false;
    }
    
    // Check if buffer is large enough for format header
    if (size < sizeof(wav_riff_header_t) + sizeof(wav_format_header_t)) {
        return false;
    }
    
    // Copy format header
    memcpy(&wav->fmt, data + sizeof(wav_riff_header_t), sizeof(wav_format_header_t));
    
    // Check format signature
    if (memcmp(wav->fmt.subchunk1_id, "fmt ", 4) != 0) {
        return false;
    }
    
    // Check if it's PCM format
    if (wav->fmt.audio_format != 1) {
        return false;
    }
    
    // Check sample format (8 or 16 bits)
    if (wav->fmt.bits_per_sample != 8 && wav->fmt.bits_per_sample != 16) {
        return false;
    }
    
    // Find data chunk
    wav_data_header_t data_header;
    bool found_data = false;
    uint32_t current_offset = sizeof(wav_riff_header_t) + sizeof(wav_format_header_t);
    
    while (!found_data) {
        // Check if buffer is large enough for subchunk header
        if (current_offset + sizeof(data_header.subchunk2_id) + sizeof(data_header.subchunk2_size) > size) {
            return false;
        }
        
        // Read subchunk header
        memcpy(&data_header.subchunk2_id, data + current_offset, sizeof(data_header.subchunk2_id));
        memcpy(&data_header.subchunk2_size, data + current_offset + sizeof(data_header.subchunk2_id), sizeof(data_header.subchunk2_size));
        
        // Check if it's data chunk
        if (memcmp(data_header.subchunk2_id, "data", 4) == 0) {
            found_data = true;
            wav->data = data_header;
            current_offset += sizeof(data_header.subchunk2_id) + sizeof(data_header.subchunk2_size);
            break;
        } else {
            // Skip other chunks
            uint32_t chunk_size = data_header.subchunk2_size;
            current_offset += sizeof(data_header.subchunk2_id) + sizeof(data_header.subchunk2_size) + chunk_size;
            
            // Check if we've gone past the buffer
            if (current_offset > size) {
                return false;
            }
        }
    }
    
    if (!found_data) {
        return false;
    }
    
    // Record data offset and set up memory mode
    wav->data_offset = current_offset;
    wav->data_pos = 0;
    wav->is_open = true;
    wav->is_memory_mode = true;
    wav->memory = data;
    wav->memory_size = size;
    
    return true;
}

bool wav_read_sample(wav_file_t *wav, int16_t *sample) {
    assert(wav && sample);
    
    if (!wav->is_open) {
        return false;
    }
    
    // If we've reached the end of data, reset to beginning
    if (wav->data_pos >= wav->data.subchunk2_size) {
        wav_reset(wav);
    }
    
    uint8_t byte_sample;
    int16_t int_sample;
    uint32_t current_pos;
    
    if (wav->is_memory_mode) {
        // Memory mode
        current_pos = wav->data_offset + wav->data_pos;
        
        if (wav->fmt.bits_per_sample == 8) {
            // 8-bit sample (unsigned)
            if (current_pos + sizeof(uint8_t) > wav->memory_size) {
                return false;
            }
            byte_sample = wav->memory[current_pos];
            // Convert to signed 16-bit
            *sample = (int16_t)((byte_sample - 128) << 8);
            wav->data_pos += sizeof(uint8_t);
        } else if (wav->fmt.bits_per_sample == 16) {
            // 16-bit sample (signed, little-endian)
            if (current_pos + sizeof(int16_t) > wav->memory_size) {
                return false;
            }
            memcpy(&int_sample, wav->memory + current_pos, sizeof(int16_t));
            *sample = int_sample;
            wav->data_pos += sizeof(int16_t);
        } else {
            // Unsupported sample format
            return false;
        }
        
        // If stereo, average channels (convert to mono)
        if (wav->fmt.num_channels == 2) {
            int16_t sample2;
            current_pos = wav->data_offset + wav->data_pos;
            
            if (wav->fmt.bits_per_sample == 8) {
                if (current_pos + sizeof(uint8_t) > wav->memory_size) {
                    return false;
                }
                byte_sample = wav->memory[current_pos];
                sample2 = (int16_t)((byte_sample - 128) << 8);
                wav->data_pos += sizeof(uint8_t);
            } else {
                if (current_pos + sizeof(int16_t) > wav->memory_size) {
                    return false;
                }
                memcpy(&sample2, wav->memory + current_pos, sizeof(int16_t));
                wav->data_pos += sizeof(int16_t);
            }
            
            // Average the two channels
            *sample = (*sample + sample2) / 2;
        }
    } else {
        // File mode
        if (!wav->file) {
            return false;
        }
        
        if (wav->fmt.bits_per_sample == 8) {
            // 8-bit sample (unsigned)
            if (fread(&byte_sample, sizeof(uint8_t), 1, wav->file) != 1) {
                return false;
            }
            // Convert to signed 16-bit
            *sample = (int16_t)((byte_sample - 128) << 8);
            wav->data_pos += sizeof(uint8_t);
        } else if (wav->fmt.bits_per_sample == 16) {
            // 16-bit sample (signed, little-endian)
            if (fread(&int_sample, sizeof(int16_t), 1, wav->file) != 1) {
                return false;
            }
            *sample = int_sample;
            wav->data_pos += sizeof(int16_t);
        } else {
            // Unsupported sample format
            return false;
        }
        
        // If stereo, average channels (convert to mono)
        if (wav->fmt.num_channels == 2) {
            int16_t sample2;
            
            if (wav->fmt.bits_per_sample == 8) {
                if (fread(&byte_sample, sizeof(uint8_t), 1, wav->file) != 1) {
                    return false;
                }
                sample2 = (int16_t)((byte_sample - 128) << 8);
                wav->data_pos += sizeof(uint8_t);
            } else {
                if (fread(&sample2, sizeof(int16_t), 1, wav->file) != 1) {
                    return false;
                }
                wav->data_pos += sizeof(int16_t);
            }
            
            // Average the two channels
            *sample = (*sample + sample2) / 2;
        }
    }
    
    return true;
}

void wav_close(wav_file_t *wav) {
    assert(wav);
    
    if (wav->is_open && wav->file) {
        fclose(wav->file);
        wav->is_open = false;
        wav->file = NULL;
    }
}

void wav_reset(wav_file_t *wav) {
    assert(wav);
    
    if (wav->is_open) {
        if (wav->is_memory_mode) {
            // In memory mode, just reset the data position
            wav->data_pos = 0;
        } else if (wav->file) {
            // In file mode, reset the file pointer
            fseek(wav->file, wav->data_offset, SEEK_SET);
            wav->data_pos = 0;
        }
    }
}
