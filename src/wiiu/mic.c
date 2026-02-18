#include "../config.h"
#include "wiiu.h"
#include "Limelight.h"
#include <malloc.h>
#include <string.h>
#include "mic/mic.h"
#include <coreinit/time.h>
#include <coreinit/alarm.h>
#include <coreinit/thread.h>
#include <stdlib.h>
#include <whb/log.h>
#include <sys/socket.h>
#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "../vban.h"

#define SAMPLE_MAX_COUNT 0x2800
#define SAMPLE_RATE 32000
#define MIC_UPDATE_RATE_MS 16  // 60Hz for mic

// Module-local state
static MICHandle micHandle;
static MICError error;
static MICStatus micStatus;
static MICWorkMemory workMemory;
static int16_t *sampleBuffer;

static VBAN_HEADER vban_header;
static int udp_socket = -1;
static struct sockaddr_in server_addr;

static int mic_thread_running;
static OSThread micThread;
static OSAlarm micAlarm;

static int mic_button_pressed = 0;

// External config variables (defined in input.c)
int mic_button_key = 162; //left control = 162
int mic_threshold = 2000;

int vban_enable = 0;
char* vban_name = "WiiUMic";
int vban_samplerate = 1;
int vban_bitdepth = 0;
char* vban_ipaddress = NULL;
int vban_port = 6980;

static void initializeMic(void) {
  sampleBuffer = (int16_t *)memalign(0x40, SAMPLE_MAX_COUNT * sizeof(int16_t));
  memset(sampleBuffer, 0, SAMPLE_MAX_COUNT * sizeof(int16_t));
  workMemory.sampleMaxCount = SAMPLE_MAX_COUNT;
  workMemory.sampleBuffer = sampleBuffer; 
  micHandle = MICInit(MIC_INSTANCE_0, 0, &workMemory, &error);
  error = MICOpen(micHandle);
}

static void init_vban_sender(const char *server_ip, uint16_t server_port, 
                             const char *stream_name, uint8_t num_channels) {
    if (!server_ip || !stream_name) {
        WHBLogPrintf("VBAN: Invalid parameters - NULL pointer");
        return;
    }
    
    vban_init_header(&vban_header, stream_name, num_channels, 1);
    
    udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_socket < 0) {
        WHBLogPrintf("VBAN: Failed to create UDP socket");
        return;
    }
    
    WHBLogPrintf("VBAN: UDP socket created successfully");
    
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(server_port);
    
    int inet_result = inet_pton(AF_INET, server_ip, &server_addr.sin_addr);
    if (inet_result <= 0) {
        WHBLogPrintf("VBAN: inet_pton failed for IP: %s (result: %d)", server_ip, inet_result);
        udp_socket = -1;
        return;
    }
    
    WHBLogPrintf("VBAN: Initialized sender - IP: %s, Port: %u, Stream: %s, Channels: %u", 
                 server_ip, server_port, stream_name, num_channels);
}

static void handleMic(void) {
  error = MICGetStatus(micHandle, &micStatus);

  uint32_t sample_count = micStatus.availableData;

  if (sample_count == 0) {
    return;
  }

  int16_t *audio_data = malloc(sample_count * sizeof(int16_t));
  if (!audio_data) {
    return;
  }

  for (uint32_t i = 0; i < sample_count; i++) {
    int idx = (micStatus.bufferPos - sample_count + i + SAMPLE_MAX_COUNT) %
              SAMPLE_MAX_COUNT;
    audio_data[i] = sampleBuffer[idx];
  }

  if (mic_button_key != 0) {
    int64_t sum = 0;
    for (uint32_t i = 0; i < sample_count; i++) {
      sum += abs(audio_data[i]);
    }
    int16_t avg_amplitude = (int16_t)(sum / sample_count);

    if (avg_amplitude > mic_threshold) {
      if (!mic_button_pressed) {
        mic_button_pressed = 1;
        LiSendKeyboardEvent(mic_button_key, KEY_ACTION_DOWN, 0);
      }
    } else {
      if (mic_button_pressed) {
        mic_button_pressed = 0;
        LiSendKeyboardEvent(mic_button_key, KEY_ACTION_UP, 0);
      }
    }
  }

  if (vban_enable) {
    const uint32_t VBAN_MAX_SAMPLES = 256;
    uint32_t samples_sent = 0;
    
    while (samples_sent < sample_count) {
        uint32_t samples_this_frame = sample_count - samples_sent;
        if (samples_this_frame > VBAN_MAX_SAMPLES) {
            samples_this_frame = VBAN_MAX_SAMPLES;
        }
        
        uint32_t bytes_per_sample = vban_bitdepth ? 1 : 2;
        uint32_t packet_size_max = sizeof(VBAN_HEADER) + (samples_this_frame * bytes_per_sample);
        uint8_t *vban_packet = malloc(packet_size_max);
        if (!vban_packet) {
            free(audio_data);
            return;
        }
        
        uint32_t packet_size = vban_pack_audio(vban_packet, 
                                               &audio_data[samples_sent],
                                               samples_this_frame, 1, 
                                               &vban_header, vban_bitdepth, 
                                               vban_samplerate);
        
        sendto(udp_socket, vban_packet, packet_size, 0,
               (struct sockaddr *)&server_addr, sizeof(server_addr));
        
        vban_next_frame(&vban_header);
        
        samples_sent += samples_this_frame;
        free(vban_packet);
    }
  }

  free(audio_data);
  error = MICSetDataConsumed(micHandle, sample_count);
}

static void mic_alarm_callback(OSAlarm* alarm, OSContext* ctx)
{
  if (mic_button_key != 0 || vban_enable) {
    handleMic();
  }
}

static int mic_thread_proc(int argc, const char **argv)
{
  OSCreateAlarm(&micAlarm);
  OSSetPeriodicAlarm(&micAlarm, 0, OSMillisecondsToTicks(MIC_UPDATE_RATE_MS), mic_alarm_callback);
  
  while (mic_thread_running) {
    OSWaitAlarm(&micAlarm);
  }
  
  return 0;
}

static void thread_deallocator(OSThread *thread, void *stack)
{
  free(stack);
}

void wiiu_mic_start(void)
{
  if (!(vban_enable || mic_button_key != 0)) {
    return;
  }

  initializeMic();
  WHBLogPrintf("Initialized microphone");
  
  if (vban_enable) { 
    WHBLogPrintf("Starting VBAN sender with IP: %s, Port: %u, Stream Name: %s, Sample Rate: %u, Bit Depth: %u",
                 vban_ipaddress, vban_port, vban_name, vban_samplerate ? 16000 : 32000, vban_bitdepth ? 8 : 16);
    init_vban_sender(vban_ipaddress, vban_port, vban_name, 1);
    WHBLogPrintf("Initialized VBAN sender");
  }
  
  const int mic_stack_size = 1 * 1024 * 1024;
  uint8_t* mic_stack = (uint8_t*)memalign(16, mic_stack_size);
  if (!mic_stack) {
    WHBLogPrintf("Failed to allocate mic thread stack");
    return;
  }

  if (OSCreateThread(&micThread,
                    mic_thread_proc,
                    0, NULL,
                    mic_stack + mic_stack_size, mic_stack_size,
                    0x10, OS_THREAD_ATTRIB_AFFINITY_CPU1))
  {
    mic_thread_running = 1;
    OSSetThreadName(&micThread, "MicInput");
    OSSetThreadDeallocator(&micThread, thread_deallocator);
    OSResumeThread(&micThread);
    WHBLogPrintf("Mic thread started");
  } else {
    free(mic_stack);
    WHBLogPrintf("Failed to create mic thread");
  }
}

void wiiu_mic_stop(void)
{
  if (mic_thread_running) {
    mic_thread_running = 0;
    OSCancelAlarm(&micAlarm);
    OSJoinThread(&micThread, NULL);
  }

  if (vban_enable || mic_button_key != 0) {
    MICClose(micHandle);
    MICUninit(micHandle);
  }

  if (vban_enable && udp_socket >= 0) {
    close(udp_socket);
    udp_socket = -1;
  }
}