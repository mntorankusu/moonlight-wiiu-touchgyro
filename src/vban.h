#ifndef VBAN_AUDIO_H
#define VBAN_AUDIO_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

// VBAN Header structure (28 bytes)
typedef struct {
  uint32_t vban;       // 'V' 'B' 'A' 'N'
  uint8_t format_SR;   // Sample rate index
  uint8_t format_nbs;  // Samples per frame - 1 (0-255 = 1-256 samples)
  uint8_t format_nbc;  // Channels - 1 (0 = 1 channel, 1 = 2 channels, etc.)
  uint8_t format_bit;  // Bit format (0 = 8-bit, 1 = 16-bit)
  char streamname[16]; // Stream name (null-terminated)
  uint32_t nuFrame;    // Frame counter
} __attribute__((packed)) VBAN_HEADER;

// VBAN Sample Rate indices
#define VBAN_SR_8000 7
#define VBAN_SR_16000 8
#define VBAN_SR_32000 9

// Byte swap helpers
static inline uint32_t bswap32(uint32_t x) {
  return ((x << 24) & 0xFF000000) | ((x << 8) & 0x00FF0000) |
         ((x >> 8) & 0x0000FF00) | ((x >> 24) & 0x000000FF);
}

static inline int16_t dither_sample(int16_t sample, uint32_t frame,
                                    uint32_t sample_index) {
  // Simple pseudo-random dithering using frame and sample index
  // This creates a deterministic but noise-like pattern
  uint32_t seed = (frame * 65599) ^ (sample_index * 12345);
  seed = (seed >> 16) ^ (seed & 0xFFFF);
  int16_t dither = ((seed & 0x07) - 4);  // ±4
  return sample + dither;
}

static inline uint16_t bswap16(uint16_t x) { return (x << 8) | (x >> 8); }

// Initialize VBAN header
static inline void vban_init_header(VBAN_HEADER *header,
                                    const char *stream_name,
                                    uint8_t num_channels,
                                    uint16_t samples_per_frame) {
  // Write magic bytes explicitly (endian-agnostic)
  uint8_t *magic = (uint8_t *)&header->vban;
  magic[0] = 'V';
  magic[1] = 'B';
  magic[2] = 'A';
  magic[3] = 'N';

  header->format_SR = VBAN_SR_32000;
  header->format_nbs = (uint8_t)(samples_per_frame - 1);
  header->format_nbc = num_channels - 1;
  header->format_bit = 1; // 16-bit default
  header->nuFrame = 0;

  memset(header->streamname, 0, 16);
  if (stream_name) {
    strncpy(header->streamname, stream_name, 16);
  }
}

// Pack audio data into VBAN packet
// audio_samples: Always expects int16_t* input (16-bit samples)
// is_8bit: If true, converts to 8-bit internally before packing
// downsample: 0=no downsampling, 2=downsample to 16kHz, 4=downsample to 8kHz
//            Uses averaging instead of simple sample dropping for smoother
//            audio
static inline int vban_pack_audio(uint8_t *packet, const int16_t *audio_samples,
                                  uint32_t num_samples, uint8_t num_channels,
                                  VBAN_HEADER *header, uint8_t is_8bit,
                                  uint8_t downsample) {

  uint32_t step = downsample ? downsample : 1;
  uint32_t effective_samples = num_samples / step;
  uint8_t bytes_per_sample = is_8bit ? 1 : 2;
  uint32_t audio_size = effective_samples * num_channels * bytes_per_sample;

  // Write header with proper byte order
  uint32_t *pkt32 = (uint32_t *)packet;
  uint8_t *pkt8 = packet;

  // Magic bytes
  pkt32[0] = header->vban;
  pkt8[4] = (downsample == 4)   ? VBAN_SR_8000
            : (downsample == 2) ? VBAN_SR_16000
                                : VBAN_SR_32000;
  pkt8[5] = (uint8_t)(effective_samples - 1);
  pkt8[6] = header->format_nbc;
  pkt8[7] = is_8bit ? 0 : 1;
  memcpy(&pkt8[8], header->streamname, 16);
  pkt32[6] = bswap32(header->nuFrame); // Frame counter to little-endian

  // Copy and process audio data
  uint8_t *audio_dst = packet + sizeof(VBAN_HEADER);

  if (!downsample) {
    // No downsampling - just convert and copy
    if (is_8bit) {
      for (uint32_t i = 0; i < num_samples * num_channels; i++) {
        int16_t sample = audio_samples[i];
        audio_dst[i] = (uint8_t)((sample + 32768) >> 8);
      }
    } else {
      uint16_t *dst = (uint16_t *)audio_dst;
      for (uint32_t i = 0; i < num_samples * num_channels; i++) {
        dst[i] = bswap16((uint16_t)audio_samples[i]);
      }
    }
  } else {
    // Downsampling with averaging
    if (is_8bit) {
      // Convert to 8-bit with averaging and dithering
      for (uint32_t i = 0; i < effective_samples * num_channels; i++) {
        int32_t sum = 0;
        for (uint32_t j = 0; j < step; j++) {
          sum += audio_samples[i * step + j];
        }
        int16_t avg_sample = (int16_t)(sum / step);

        // Apply dithering before quantization
        int16_t dithered = dither_sample(avg_sample, header->nuFrame, i);

        // Clamp to valid range before converting
        int32_t clamped = dithered + 32768;
        if (clamped < 0)
          clamped = 0;
        if (clamped > 65535)
          clamped = 65535;

        audio_dst[i] = (uint8_t)(clamped >> 8);
      }
    } else {
      // 16-bit with averaging
      uint16_t *dst = (uint16_t *)audio_dst;
      for (uint32_t i = 0; i < effective_samples * num_channels; i++) {
        int32_t sum = 0;
        for (uint32_t j = 0; j < step; j++) {
          sum += audio_samples[i * step + j];
        }
        int16_t avg_sample = (int16_t)(sum / step);
        dst[i] = bswap16((uint16_t)avg_sample);
      }
    }
  }

  return sizeof(VBAN_HEADER) + audio_size;
}

// Increment frame counter
static inline void vban_next_frame(VBAN_HEADER *header) { header->nuFrame++; }

#endif // VBAN_AUDIO_H