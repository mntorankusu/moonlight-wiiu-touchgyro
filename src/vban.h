#ifndef VBAN_AUDIO_H
#define VBAN_AUDIO_H

#include <stdint.h>
#include <string.h>
#include <stddef.h>

// VBAN Header structure
typedef struct {
    uint32_t vban;           /* contains 'V' 'B', 'A', 'N' */
    uint8_t format_SR;       /* SR index (see SRList above) */
    uint8_t format_nbs;      /* nb sample per frame (1 to 256) */
    uint8_t format_nbc;      /* nb channel (1 to 256) */
    uint8_t format_bit;      /* mask = 0x07 (nb Byte integer from 1 to 4) */
    char streamname[16];     /* stream name */
    uint32_t nuFrame;        /* growing frame number. */
} __attribute__((packed)) VBAN_HEADER;

// VBAN Sample Rate indices
#define VBAN_SR_INVALID    0xFF
#define VBAN_SR_8000       7
#define VBAN_SR_16000      8
#define VBAN_SR_32000      9

// Helper function to swap endianness of a 32-bit value
static inline uint32_t swap_endian32(uint32_t value) {
    return ((value & 0xFF) << 24) |
           ((value & 0xFF00) << 8) |
           ((value & 0xFF0000) >> 8) |
           ((value >> 24) & 0xFF);
}

// Helper function to swap endianness of a 16-bit value
static inline uint16_t swap_endian16(uint16_t value) {
    return ((value & 0xFF) << 8) | ((value >> 8) & 0xFF);
}

// Initialize VBAN header in native (big-endian) format
void vban_init_header(VBAN_HEADER *header, const char *stream_name, 
                      uint8_t num_channels, uint16_t samples_per_frame) {
    // Set VBAN magic number in big-endian (native Wii U format)
    header->vban = 0x5642414E;
    
    header->format_SR = VBAN_SR_32000;
    header->format_nbs = (uint8_t)(samples_per_frame - 1);
    header->format_nbc = num_channels - 1;  // 0 = 1 channel, 1 = 2 channels, etc.
    header->format_bit = 1;                 // 16-bit PCM
    
    // Initialize stream name - pad with zeros, NO null terminator
    memset(header->streamname, 0, 16);
    if (stream_name) {
        strncpy(header->streamname, stream_name, 16);
    }
    
    header->nuFrame = 0;
    
    // Debug: Log actual struct offsets (only once)
    static int logged = 0;
    if (!logged) {
        WHBLogPrintf("VBAN: Header structure offsets:");
        WHBLogPrintf("  Offset of vban: %zu", offsetof(VBAN_HEADER, vban));
        WHBLogPrintf("  Offset of format_SR: %zu", offsetof(VBAN_HEADER, format_SR));
        WHBLogPrintf("  Offset of format_nbs: %zu", offsetof(VBAN_HEADER, format_nbs));
        WHBLogPrintf("  Offset of format_nbc: %zu", offsetof(VBAN_HEADER, format_nbc));
        WHBLogPrintf("  Offset of format_bit: %zu", offsetof(VBAN_HEADER, format_bit));
        WHBLogPrintf("  Offset of streamname: %zu", offsetof(VBAN_HEADER, streamname));
        WHBLogPrintf("  Offset of nuFrame: %zu", offsetof(VBAN_HEADER, nuFrame));
        WHBLogPrintf("  Total header size: %zu", sizeof(VBAN_HEADER));
        logged = 1;
    }
}

// Pack audio data into VBAN packet with variable sample count and bit depth
int vban_pack_audio(uint8_t *packet, const void *audio_samples,
                    uint32_t num_samples, uint8_t num_channels,
                    VBAN_HEADER *header, uint8_t is_8bit, uint8_t downsample) {
    WHBLogPrintf("VBAN: vban_pack_audio called with num_samples = %u, %u-bit", 
                 num_samples, is_8bit ? 8 : 16);
    
    // Step 1: Create header in native (big-endian) format
    VBAN_HEADER native_header;
    memcpy(&native_header, header, sizeof(VBAN_HEADER));
    
    // Apply downsampling if requested
    uint32_t effective_samples = num_samples;
    
    if (downsample == 2) {
        effective_samples = num_samples / 2;
        native_header.format_SR = VBAN_SR_16000;  // Half sample rate
    } else if (downsample == 4) {
        effective_samples = num_samples / 4;
        native_header.format_SR = VBAN_SR_8000;  // 1/4 sample rate
    } else {
        native_header.format_SR = VBAN_SR_32000;  // Original sample rate
    }
    
    native_header.format_nbs = (uint8_t)(effective_samples - 1);
    native_header.format_bit = is_8bit ? 0 : 1;  // 0 = 8-bit, 1 = 16-bit
    
    // Step 2: Write header fields to packet with proper endianness
    uint8_t *pkt = packet;
    
    // Magic number
    pkt[0] = 0x56;
    pkt[1] = 0x42;
    pkt[2] = 0x41;
    pkt[3] = 0x4E;
    
    // Single-byte fields
    pkt[4] = native_header.format_SR;
    pkt[5] = native_header.format_nbs;
    pkt[6] = native_header.format_nbc;
    pkt[7] = native_header.format_bit;
    
    // Stream name
    memcpy(&pkt[8], native_header.streamname, 16);
    
    // Frame number - convert to little-endian
    uint32_t frame_le = swap_endian32(native_header.nuFrame);
    memcpy(&pkt[24], &frame_le, sizeof(uint32_t));
    
    // Step 3: Copy audio samples
    uint32_t bytes_per_sample = is_8bit ? 1 : 2;
    uint32_t audio_data_size = effective_samples * num_channels * bytes_per_sample;
    
    if (is_8bit) {
        // 8-bit data
        const uint8_t *audio_input = (const uint8_t *)audio_samples;
        uint8_t *audio_output = packet + sizeof(VBAN_HEADER);
        if (downsample) {
            // Skip every other sample
            for (uint32_t i = 0; i < effective_samples * num_channels; i++) {
                audio_output[i] = audio_input[i * downsample];
            }
        } else {
            memcpy(audio_output, audio_samples, audio_data_size);
        }
    } else {
        // 16-bit data needs byte swapping
        uint16_t *audio_swap = (uint16_t *)(packet + sizeof(VBAN_HEADER));
        const int16_t *audio_input = (const int16_t *)audio_samples;
        if (downsample) {
            // Skip every other sample while byte swapping
            for (uint32_t i = 0; i < effective_samples * num_channels; i++) {
                audio_swap[i] = swap_endian16((uint16_t)audio_input[i * downsample]);
            }
        } else {
            for (uint32_t i = 0; i < num_samples * num_channels; i++) {
                audio_swap[i] = swap_endian16((uint16_t)audio_input[i]);
            }
        }
    }
    
    // Step 4: Calculate total packet size
    uint32_t total_size = sizeof(VBAN_HEADER) + audio_data_size;
    
    // Debug logging (first frame only)
    static int first_frame = 1;
    if (first_frame) {
        WHBLogPrintf("VBAN: 0x%02X%02X%02X%02X : 0x%02X%02X%02X%02X : 0x%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X : 0x%02X%02X%02X%02X",
                     pkt[0], pkt[1], pkt[2], pkt[3],
                     pkt[4], pkt[5], pkt[6], pkt[7],
                     pkt[8], pkt[9], pkt[10], pkt[11], pkt[12], pkt[13], pkt[14], pkt[15],
                     pkt[16], pkt[17], pkt[18], pkt[19], pkt[20], pkt[21], pkt[22], pkt[23],
                     pkt[24], pkt[25], pkt[26], pkt[27]);
        first_frame = 0;
    }
    
    return total_size;
}

// Increment frame counter
void vban_next_frame(VBAN_HEADER *header) {
    header->nuFrame++;
    // Frame counter wraps at 32-bit boundary
}

#endif // VBAN_AUDIO_H