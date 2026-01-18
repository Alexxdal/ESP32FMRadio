/* minimp3 - single header MP3 decoder library v1.29

   LICENSE

   minimp3 is licensed under the following terms, except for the parts that are
   copied from KISS FFT.

   Copyright (c) 2017-2021, Vitaliy E. Palchevskiy. All rights reserved.
   Contacts: vitaliy.palchevskiy@gmail.com

   Permission to use, copy, modify, and distribute this software for any
   purpose with or without fee is hereby granted, provided that the above
   copyright notice and this permission notice appear in all copies.

   THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
   WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
   MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
   ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
   WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
   ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
   OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

   --- KISS FFT --- Copyright (c) 2003-2010, Mark Borgerding

   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.
   * Neither the author nor the names of any contributors may be used to
     endorse or promote products derived from this software without specific
     prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef MINIMP3_H
#define MINIMP3_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define MINIMP3_MAX_SAMPLES_PER_FRAME 1152

typedef struct {
    uint8_t *frame; /* pointer to internal frame storage */
    uint8_t channels;
    uint32_t sample_rate;
    int frame_bytes; /* single frame size in bytes */
    int audio_bytes; /* number of bytes consumed from input buffer to produce current sample count */
    int layer;
    int avg_bitrate_kbps;
    int bitrate_kbps;
    int buffer_consumed;
    int frame_offset;
    int free_format_bytes;
    int free_format_next_header_bytes;
    int free_format_frames; /* 0 - unknown, -1 - not free format, >0 - estimated number of frames */
    int samples; /* number of output samples per channel */
    /* internal data */
    void *decode_buf;
    void *kiss_fft_state;
    void *huffman_table[2];
    int32_t mdct_coeff[2][2][16][36];
    int32_t synth_buf[2][MINIMP3_MAX_SAMPLES_PER_FRAME];
    int error;
    int layer_num; /* last decoded layer */
    int frame_bytes_left;
    uint8_t *frame_left; /* leftover from previous frame */
    int free_format_frames_left;
    int free_format_bitrate_kbps;
    int last_frame_bytes; /* used for Xing LAME free format bitrate calculation */
} mp3dec_t;

typedef enum {
    MINIMP3_OK = 0,
    MINIMP3_ERROR_OUT_OF_MEMORY = -1,
    MINIMP3_ERROR_IO = -2,
    MINIMP3_ERROR_INVALID_FILE = -3,
    MINIMP3_ERROR_UNSUPPORTED_LAYER = -4,
    MINIMP3_ERROR_BAD_FRAME_HEADER = -5,
    MINIMP3_ERROR_DECODE_ERROR = -6,
} mp3dec_status_t;

typedef struct {
    int header_pos;
    int frame_bytes;
    int channels;
    int layer;
    int sample_rate;
    int bitrate_kbps;
    int error;
} mp3dec_frame_info_t;

typedef struct {
    int samples; /* number of samples decoded so far */
    int frames; /* number of frames decoded so far */
    int max_frame_bytes; /* maximum frame size encountered */
    int avg_bitrate_kbps;
    int sample_rate;
    int channels;
    int layer;
    int total_samples; /* total estimated samples, zero if VBR or unknown */
    int total_frames; /* total estimated frames, zero if unknown */
} mp3dec_stats_t;

void mp3dec_init(mp3dec_t *dec);
int mp3dec_load(mp3dec_t *dec, const uint8_t *buf, int buf_size, int16_t *pcm, mp3dec_frame_info_t *info);
int mp3dec_decode_frame(mp3dec_t *dec, const uint8_t *buf, int buf_size, int16_t *pcm, mp3dec_frame_info_t *info);
void mp3dec_get_info(mp3dec_t *dec, mp3dec_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif
