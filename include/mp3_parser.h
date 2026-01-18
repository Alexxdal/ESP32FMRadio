#ifndef MP3_PARSER_H
#define MP3_PARSER_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

// MP3 file context
typedef struct {
    FILE *file;              // File pointer (for file mode)
    const uint8_t *memory;   // Memory pointer (for memory mode)
    uint32_t sample_rate;    // Sample rate (e.g., 8000, 44100)
    uint16_t num_channels;   // Number of channels (1 = mono, 2 = stereo)
    uint32_t bitrate;        // Bitrate in kbps
    uint32_t data_pos;       // Current position in data
    uint32_t memory_size;    // Total size of memory buffer (for memory mode)
    bool is_open;            // Whether file is open
    bool is_memory_mode;     // Whether using memory mode
    bool has_id3v1;          // Whether the file has ID3v1 tag
    bool has_id3v2;          // Whether the file has ID3v2 tag
    uint32_t id3v2_size;     // Size of ID3v2 tag
} mp3_file_t;

/**
 * Open an MP3 file and parse its headers
 * @param filename Path to MP3 file
 * @param mp3 Pointer to mp3_file_t structure to populate
 * @return true on success, false on failure
 */
bool mp3_open(const char *filename, mp3_file_t *mp3);

/**
 * Open an MP3 file from memory and parse its headers
 * @param data Pointer to MP3 data in memory
 * @param size Size of MP3 data in bytes
 * @param mp3 Pointer to mp3_file_t structure to populate
 * @return true on success, false on failure
 */
bool mp3_open_from_memory(const uint8_t *data, uint32_t size, mp3_file_t *mp3);

/**
 * Read a single sample from MP3 file
 * @param mp3 Pointer to open MP3 file context
 * @param sample Pointer to store the read sample (-32768 to 32767)
 * @return true on success, false on end of file or error
 */
bool mp3_read_sample(mp3_file_t *mp3, int16_t *sample);

/**
 * Close an MP3 file
 * @param mp3 Pointer to MP3 file context
 */
void mp3_close(mp3_file_t *mp3);

/**
 * Reset file pointer to beginning of audio data
 * @param mp3 Pointer to MP3 file context
 */
void mp3_reset(mp3_file_t *mp3);

/**
 * Check if a file is an MP3 file
 * @param filename Path to file
 * @return true if MP3 file, false otherwise
 */
bool is_mp3_file(const char *filename);

/**
 * Check if data in memory is an MP3 file
 * @param data Pointer to data in memory
 * @param size Size of data in bytes
 * @return true if MP3 data, false otherwise
 */
bool is_mp3_data(const uint8_t *data, uint32_t size);

#endif // MP3_PARSER_H