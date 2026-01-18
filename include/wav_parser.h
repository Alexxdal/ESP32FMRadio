#ifndef WAV_PARSER_H
#define WAV_PARSER_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

// WAV file header structures
#pragma pack(push, 1)

// RIFF header
typedef struct {
    char chunk_id[4];        // "RIFF"
    uint32_t chunk_size;     // File size - 8 bytes
    char format[4];          // "WAVE"
} wav_riff_header_t;

// Format chunk
typedef struct {
    char subchunk1_id[4];    // "fmt "
    uint32_t subchunk1_size; // Format chunk size
    uint16_t audio_format;   // Audio format (1 = PCM)
    uint16_t num_channels;   // Number of channels (1 = mono, 2 = stereo)
    uint32_t sample_rate;    // Sample rate (e.g., 8000, 44100)
    uint32_t byte_rate;      // Byte rate = sample_rate * num_channels * bits_per_sample / 8
    uint16_t block_align;    // Block align = num_channels * bits_per_sample / 8
    uint16_t bits_per_sample;// Bits per sample (e.g., 8, 16)
} wav_format_header_t;

// Data chunk header
typedef struct {
    char subchunk2_id[4];    // "data"
    uint32_t subchunk2_size; // Data size in bytes
} wav_data_header_t;

#pragma pack(pop)

// WAV file context
typedef struct {
    FILE *file;              // File pointer (for file mode)
    const uint8_t *memory;   // Memory pointer (for memory mode)
    wav_riff_header_t riff;  // RIFF header
    wav_format_header_t fmt; // Format header
    wav_data_header_t data;  // Data header
    uint32_t data_offset;    // Offset to data start
    uint32_t data_pos;       // Current position in data
    uint32_t memory_size;    // Total size of memory buffer (for memory mode)
    bool is_open;            // Whether file is open
    bool is_memory_mode;     // Whether using memory mode
} wav_file_t;

/**
 * Open a WAV file and parse its headers
 * @param filename Path to WAV file
 * @param wav Pointer to wav_file_t structure to populate
 * @return true on success, false on failure
 */
bool wav_open(const char *filename, wav_file_t *wav);

/**
 * Open a WAV file from memory and parse its headers
 * @param data Pointer to WAV data in memory
 * @param size Size of WAV data in bytes
 * @param wav Pointer to wav_file_t structure to populate
 * @return true on success, false on failure
 */
bool wav_open_from_memory(const uint8_t *data, uint32_t size, wav_file_t *wav);

/**
 * Read a single sample from WAV file
 * @param wav Pointer to open WAV file context
 * @param sample Pointer to store the read sample (0-255 for 8-bit, -32768 to 32767 for 16-bit)
 * @return true on success, false on end of file or error
 */
bool wav_read_sample(wav_file_t *wav, int16_t *sample);

/**
 * Close a WAV file
 * @param wav Pointer to WAV file context
 */
void wav_close(wav_file_t *wav);

/**
 * Reset file pointer to beginning of audio data
 * @param wav Pointer to WAV file context
 */
void wav_reset(wav_file_t *wav);

#endif // WAV_PARSER_H
