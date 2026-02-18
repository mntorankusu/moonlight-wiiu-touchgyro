#include "../config.h"
#include "wiiu.h"
#include "Limelight.h"
#include <nn/acp/drcled_c.h>
#include <malloc.h>
#include <string.h>
#include <vpad/input.h>
#include "mic/mic.h"
#include <padscore/kpad.h>
#include <padscore/wpad.h>
#include <coreinit/time.h>
#include <coreinit/alarm.h>
#include <coreinit/thread.h>
#include <nn/act/client_cpp.h>
#include <stdlib.h>
#include <whb/log.h>
#include <whb/log_cafe.h>
#include <whb/log_udp.h>
#include <sys/socket.h>
#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "../vban.h" 

#define millis() OSTicksToMilliseconds(OSGetTime())

int disable_gamepad = 0;
int swap_buttons = 0;
mouse_modes mouse_mode = MOUSE_MODE_RELATIVE;

int gyro_output = 0;
int gyro_magnification = 1;
int power_button_key = 161; //p key = 80
int mic_button_key = 163; //right control = 163
int mic_threshold = 2000;
float input_update_rate = 16.0f;
int vban_enable = 0;
int vban_samplerate = 1;
int vban_bitdepth = 0;
char* vban_ipaddress = NULL;
int vban_port = 6980;

int use_8bit = 1;

static char lastTouched = 0;
static char touched = 0;
static int rumble_weak = 0;
static int rumble_strong = 0;
static uint32_t power_button_pressed = 0;
static int mic_button_pressed = 0;

MICHandle micHandle;
MICError error;
MICStatus micStatus;
MICWorkMemory workMemory;
int16_t *sampleBuffer;

static VBAN_HEADER vban_header;
static uint32_t frame_count = 0;
static int udp_socket = -1;
static struct sockaddr_in server_addr;

uint8_t rumblepattern1[12] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t rumblepattern2[12] = { 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint16_t last_x = 0;
static uint16_t last_y = 0;

static uint8_t batteryLevel = 255;
static uint8_t lastBatteryLevel = 0;

#define TAP_MILLIS 100
#define DRAG_DISTANCE 10
static uint64_t touchDownMillis = 0;

#define TOUCH_WIDTH 1920.0f
#define TOUCH_HEIGHT 1080.0f
#define VPAD_BUTTON_POWER 0x00080000

#define SAMPLE_MAX_COUNT 0x2800
#define SAMPLE_RATE 32000
#define DOWNSAMPLE_16000 2
#define DOWNSAMPLE_8000 4
#define BIT_DEPTH_8 1
#define BIT_DEPTH_16 0


static int thread_running;
static OSThread inputThread;
static OSAlarm inputAlarm;

//Lower numbers mean lower latency, but there may be an impact on performance. 4ms is 250hz, so it's unlikely that any lower value will help much.
#define INPUT_UPDATE_RATE OSMillisecondsToTicks(input_update_rate)

void initializeMic() {
  sampleBuffer = (int16_t *)memalign(0x40, SAMPLE_MAX_COUNT * sizeof(int16_t));
  memset(sampleBuffer, 0, SAMPLE_MAX_COUNT * sizeof(int16_t));
  workMemory.sampleMaxCount = SAMPLE_MAX_COUNT;
  workMemory.sampleBuffer = sampleBuffer; 
  micHandle = MICInit(MIC_INSTANCE_0, 0, &workMemory, &error);
  error = MICOpen(micHandle);
}

int lastBufferPos = 0;

void init_vban_sender(const char *server_ip, uint16_t server_port, 
                      const char *stream_name, uint8_t num_channels) {
    if (!server_ip || !stream_name) {
        WHBLogPrintf("VBAN: Invalid parameters - NULL pointer");
        return;
    }
    
    // Initialize VBAN header with a default value (will be updated per packet)
    vban_init_header(&vban_header, stream_name, num_channels, 1);
    
    // Setup UDP socket
    udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_socket < 0) {
        WHBLogPrintf("VBAN: Failed to create UDP socket");
        return;
    }
    
    WHBLogPrintf("VBAN: UDP socket created successfully");
    
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(server_port);
    
    // Use inet_pton instead of inet_aton for better error handling
    int inet_result = inet_pton(AF_INET, server_ip, &server_addr.sin_addr);
    if (inet_result <= 0) {
        WHBLogPrintf("VBAN: inet_pton failed for IP: %s (result: %d)", server_ip, inet_result);
        udp_socket = -1;
        return;
    }
    
    WHBLogPrintf("VBAN: Initialized sender - IP: %s, Port: %u, Stream: %s, Channels: %u", 
                 server_ip, server_port, stream_name, num_channels);
}

void handleMic() {
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
        int idx = (micStatus.bufferPos - sample_count + i + SAMPLE_MAX_COUNT) % SAMPLE_MAX_COUNT;
        audio_data[i] = sampleBuffer[idx];
    }
    
    uint8_t *audio_data_8bit = NULL;
    if (vban_bitdepth == BIT_DEPTH_8) {
        audio_data_8bit = malloc(sample_count * sizeof(uint8_t));
        if (!audio_data_8bit) {
            free(audio_data);
            return;
        }
        
        for (uint32_t i = 0; i < sample_count; i++) {
          int16_t sample = audio_data[i];
          audio_data_8bit[i] = (uint8_t)((sample + 32768) >> 8);
        }
    }

    // Split into VBAN frames (max 256 samples per frame)
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
            if (vban_bitdepth == BIT_DEPTH_8) {
                free(audio_data_8bit);
            }
            free(audio_data);
            return;
        }
        
        // Get pointer to the audio data for this frame
        void *frame_data = vban_bitdepth ? 
            (void *)(&audio_data_8bit[samples_sent]) :
            (void *)(&audio_data[samples_sent]);
        
        uint32_t packet_size = vban_pack_audio(vban_packet, frame_data, 
                                               samples_this_frame, 1, &vban_header, use_8bit, 2);
        
        sendto(udp_socket, vban_packet, packet_size, 0,
               (struct sockaddr *)&server_addr, sizeof(server_addr));
        
        vban_next_frame(&vban_header);
        
        samples_sent += samples_this_frame;
        free(vban_packet);
    }
    
    if (vban_bitdepth == BIT_DEPTH_8) {
        free(audio_data_8bit);
    }
    free(audio_data);
    error = MICSetDataConsumed(micHandle, sample_count);
}

uint32_t handleAdditionalKeys(uint32_t buttons) {
  static uint32_t status = 0;
  static uint32_t tick = 0;
  
    VPADBASEGetPowerButtonPressStatus(VPAD_CHAN_0, &tick, &status);
    if (status) {
      buttons |= VPAD_BUTTON_POWER;
      if (power_button_key != 0) {
        LiSendKeyboardEvent(power_button_key, KEY_ACTION_DOWN, 0);
      }
      power_button_pressed = 1;
    } else if (!status) {
      if (power_button_key != 0) {
        LiSendKeyboardEvent(power_button_key, KEY_ACTION_UP, 0);
      }
      power_button_pressed = 0;
    }

  return buttons;
}

void handleRumble(unsigned short controllerNumber, unsigned short lowFreqMotor, unsigned short highFreqMotor) {
  if (controllerNumber == 0) {
    if (lowFreqMotor < 1 && highFreqMotor > 1) {
      rumble_weak = 1;
      rumble_strong = 0;
      }
    else if (lowFreqMotor > 1) {
      rumble_weak = 0;
      rumble_strong = 1;
      } else
      {
      VPADStopMotor(VPAD_CHAN_0);
      rumble_weak = 0;
      rumble_strong = 0;
      }
    }
}

void handleMotion (VPADVec3D gyro, VPADVec3D accel) {
  if (gyro_output) {
    VPADSetGyroMagnification(VPAD_CHAN_0, 57.2958*gyro_magnification,57.2958*gyro_magnification,57.2958*gyro_magnification); // Convert from radians to degrees
    LiSendControllerMotionEvent(0, LI_MOTION_TYPE_GYRO, -gyro.x, gyro.y, gyro.z);
    //LiSendControllerMotionEvent(0, LI_MOTION_TYPE_ACCEL, -accel.x, accel.y, accel.z);
  }
}

void handleTouch(VPADTouchData touch) {
  if (mouse_mode == MOUSE_MODE_ABSOLUTE) {
    if (touch.touched) {
      LiSendMousePositionEvent(touch.x, touch.y, TOUCH_WIDTH, TOUCH_HEIGHT);

      if (!touched) {
        LiSendMouseButtonEvent(BUTTON_ACTION_PRESS, BUTTON_LEFT);
        touched = 1;
      }
    }
    else if (touched) {
      LiSendMouseButtonEvent(BUTTON_ACTION_RELEASE, BUTTON_LEFT);
      touched = 0;
    }
  }
  else if (mouse_mode == MOUSE_MODE_TOUCHSCREEN) {
    if (touch.touched) {
      if (!touched) {
        LiSendTouchEvent(LI_TOUCH_EVENT_DOWN, 0, (float) touch.x / TOUCH_WIDTH, (float) touch.y / TOUCH_HEIGHT, 0.0, 0.0, 0.0, 0.0);
        touched = 1;
      } else {
        LiSendTouchEvent(LI_TOUCH_EVENT_MOVE, 0, (float) touch.x / TOUCH_WIDTH, (float) touch.y / TOUCH_HEIGHT, 0.0, 0.0, 0.0, 0.0);
      }
    }
    else if (touched) {
      LiSendTouchEvent(LI_TOUCH_EVENT_UP, 0, (float) touch.x / TOUCH_WIDTH, (float) touch.y / TOUCH_HEIGHT, 0.0, 0.0, 0.0, 0.0);
      touched = 0;
    }
  }
  else {
    // Just pressed (run this twice to allow touch position to settle)
    if (lastTouched < 2 && touch.touched) {
      touchDownMillis = millis();
      last_x = touch.x;
      last_y = touch.y;

      lastTouched++;
      return; // We can't do much until we wait for a few hundred milliseconds
              // since we don't know if it's a tap, a tap-and-hold, or a drag
    }

    // Just released
    if (lastTouched && !touch.touched) {
      if (millis() - touchDownMillis < TAP_MILLIS) {
        LiSendMouseButtonEvent(BUTTON_ACTION_PRESS, BUTTON_LEFT);
        LiSendMouseButtonEvent(BUTTON_ACTION_RELEASE, BUTTON_LEFT);
      }
    }

    if (touch.touched) {
      // Holding & dragging screen, not just tapping
      if (millis() - touchDownMillis > TAP_MILLIS || touchDownMillis == 0) {
        if (touch.x != last_x || touch.y != last_y) // Don't send extra data if we don't need to
          LiSendMouseMoveEvent(touch.x - last_x, touch.y - last_y);
        last_x = touch.x;
        last_y = touch.y;
      } else {
        if (touch.x - last_x < -10 || touch.x - last_x > 10) touchDownMillis=0;
        if (touch.y - last_y < -10 || touch.y - last_y > 10) touchDownMillis=0;
        int16_t diff_x = touch.x - last_x;
        int16_t diff_y = touch.y - last_y;
        if (diff_x < 0) diff_x = -diff_x;
        if (diff_y < 0) diff_y = -diff_y;
        if (diff_x + diff_y > DRAG_DISTANCE) touchDownMillis = 0;
      }
    }

    lastTouched = touch.touched ? lastTouched : 0; // Keep value unless released
  }
}

void wiiu_input_init(void)
{
	KPADInit();
	WPADEnableURCC(1);
  if (!disable_gamepad) {
    VPADInitGyroZeroDriftMode(VPAD_CHAN_0);
  }
}
int arrival_set = 0;
void wiiu_input_update(void) {
  static uint64_t home_pressed[4] = {0};

  short controllerNumber = 0;
  short gamepad_mask = 0;

  for (int i = 0; i < wiiu_input_num_controllers(); i++)
    gamepad_mask |= 1 << i;

  VPADStatus vpad;
  VPADReadError err;
  VPADRead(VPAD_CHAN_0, &vpad, 1, &err);
  if (err == VPAD_READ_SUCCESS && !disable_gamepad) {
    if (!arrival_set) {
      LiSendControllerArrivalEvent(0,gamepad_mask,LI_CTYPE_PS,0x1FFFFF,LI_CCAP_RUMBLE | LI_CCAP_GYRO | LI_CCAP_BATTERY_STATE);
      arrival_set = 1;
    }
    if (rumble_strong) {
      VPADControlMotor(VPAD_CHAN_0, rumblepattern1, 8);
    } else if (rumble_weak) {
      VPADControlMotor(VPAD_CHAN_0, rumblepattern2, 10);
    } else {
      VPADStopMotor(VPAD_CHAN_0);
    }

    batteryLevel = vpad.battery;
    if (batteryLevel != lastBatteryLevel) {
      LiSendControllerBatteryEvent(0, LI_BATTERY_STATE_DISCHARGING, batteryLevel);
      lastBatteryLevel = batteryLevel;
    }

    uint32_t btns = vpad.hold;

    btns = handleAdditionalKeys(btns);

    int buttonFlags = 0;
#define CHECKBTN(v, f) if (btns & v) buttonFlags |= f;
    if (swap_buttons) {
      CHECKBTN(VPAD_BUTTON_A,       B_FLAG);
      CHECKBTN(VPAD_BUTTON_B,       A_FLAG);
      CHECKBTN(VPAD_BUTTON_X,       Y_FLAG);
      CHECKBTN(VPAD_BUTTON_Y,       X_FLAG);
    }
    else {
      CHECKBTN(VPAD_BUTTON_A,       A_FLAG);
      CHECKBTN(VPAD_BUTTON_B,       B_FLAG);
      CHECKBTN(VPAD_BUTTON_X,       X_FLAG);
      CHECKBTN(VPAD_BUTTON_Y,       Y_FLAG);
    }
    CHECKBTN(VPAD_BUTTON_UP,      UP_FLAG);
    CHECKBTN(VPAD_BUTTON_DOWN,    DOWN_FLAG);
    CHECKBTN(VPAD_BUTTON_LEFT,    LEFT_FLAG);
    CHECKBTN(VPAD_BUTTON_RIGHT,   RIGHT_FLAG);
    CHECKBTN(VPAD_BUTTON_L,       LB_FLAG);
    CHECKBTN(VPAD_BUTTON_R,       RB_FLAG);
    CHECKBTN(VPAD_BUTTON_STICK_L, LS_CLK_FLAG);
    CHECKBTN(VPAD_BUTTON_STICK_R, RS_CLK_FLAG);
    CHECKBTN(VPAD_BUTTON_PLUS,    PLAY_FLAG);
    CHECKBTN(VPAD_BUTTON_MINUS,   BACK_FLAG);
    CHECKBTN(VPAD_BUTTON_HOME,    SPECIAL_FLAG);
    CHECKBTN(VPAD_BUTTON_TV,      TOUCHPAD_FLAG);
    //CHECKBTN(VPAD_BUTTON_POWER,   MISC_FLAG);
#undef CHECKBTN

    // If the button was just pressed, reset to current time
    if (vpad.trigger & VPAD_BUTTON_HOME) home_pressed[controllerNumber] = millis();

    //Use a button combo instead of just the home button to stop the stream, since holding the home button is used for Steam chords, etc
    if (btns & VPAD_BUTTON_POWER && btns & VPAD_BUTTON_TV) {
      state = STATE_STOP_STREAM;
      return;
    }
    if (btns & VPAD_BUTTON_A) {
      char msg[100];
      sprintf(msg, "%u", vpad.slideVolume);
      WHBLogPrint(msg);
    }
    LiSendMultiControllerEvent(controllerNumber++, gamepad_mask, buttonFlags,
      (vpad.hold & VPAD_BUTTON_ZL) ? 0xFF : 0,
      (vpad.hold & VPAD_BUTTON_ZR) ? 0xFF : 0,
      vpad.leftStick.x * INT16_MAX, vpad.leftStick.y * INT16_MAX,
      vpad.rightStick.x * INT16_MAX, vpad.rightStick.y * INT16_MAX);

    VPADTouchData touch;
    VPADGetTPCalibratedPointEx(VPAD_CHAN_0, VPAD_TP_1920X1080, &touch, &vpad.tpNormal);
    
    handleMotion(vpad.gyro, vpad.accelorometer.acc);
    handleTouch(touch);
    if (mic_button_key != 0) {
      handleMic();
    }
  }

  KPADStatus kpad_data = {0};
	int32_t kpad_err = -1;
	for (int i = 0; i < 4; i++) {
		KPADReadEx((KPADChan) i, &kpad_data, 1, &kpad_err);
		if (kpad_err == KPAD_ERROR_OK && controllerNumber < 4) {
      if (kpad_data.extensionType == WPAD_EXT_PRO_CONTROLLER) {
        uint32_t btns = kpad_data.pro.hold;
        int buttonFlags = 0;
#define CHECKBTN(v, f) if (btns & v) buttonFlags |= f;
        if (swap_buttons) {
          CHECKBTN(WPAD_PRO_BUTTON_A,       B_FLAG);
          CHECKBTN(WPAD_PRO_BUTTON_B,       A_FLAG);
          CHECKBTN(WPAD_PRO_BUTTON_X,       Y_FLAG);
          CHECKBTN(WPAD_PRO_BUTTON_Y,       X_FLAG);
        }
        else {
          CHECKBTN(WPAD_PRO_BUTTON_A,       A_FLAG);
          CHECKBTN(WPAD_PRO_BUTTON_B,       B_FLAG);
          CHECKBTN(WPAD_PRO_BUTTON_X,       X_FLAG);
          CHECKBTN(WPAD_PRO_BUTTON_Y,       Y_FLAG);
        }
        CHECKBTN(WPAD_PRO_BUTTON_UP,      UP_FLAG);
        CHECKBTN(WPAD_PRO_BUTTON_DOWN,    DOWN_FLAG);
        CHECKBTN(WPAD_PRO_BUTTON_LEFT,    LEFT_FLAG);
        CHECKBTN(WPAD_PRO_BUTTON_RIGHT,   RIGHT_FLAG);
        CHECKBTN(WPAD_PRO_TRIGGER_L,      LB_FLAG);
        CHECKBTN(WPAD_PRO_TRIGGER_R,      RB_FLAG);
        CHECKBTN(WPAD_PRO_BUTTON_STICK_L, LS_CLK_FLAG);
        CHECKBTN(WPAD_PRO_BUTTON_STICK_R, RS_CLK_FLAG);
        CHECKBTN(WPAD_PRO_BUTTON_PLUS,    PLAY_FLAG);
        CHECKBTN(WPAD_PRO_BUTTON_MINUS,   BACK_FLAG);
        CHECKBTN(WPAD_PRO_BUTTON_HOME,    SPECIAL_FLAG);
#undef CHECKBTN

        // If the button was just pressed, reset to current time
        if (kpad_data.pro.trigger & WPAD_PRO_BUTTON_HOME)
          home_pressed[controllerNumber] = millis();

        if (btns & WPAD_PRO_BUTTON_HOME && millis() - home_pressed[controllerNumber] > 3000) {
          state = STATE_STOP_STREAM;
          return;
        }

        LiSendMultiControllerEvent(controllerNumber++, gamepad_mask, buttonFlags,
          (kpad_data.pro.hold & WPAD_PRO_TRIGGER_ZL) ? 0xFF : 0,
          (kpad_data.pro.hold & WPAD_PRO_TRIGGER_ZR) ? 0xFF : 0,
          kpad_data.pro.leftStick.x * INT16_MAX, kpad_data.pro.leftStick.y * INT16_MAX,
          kpad_data.pro.rightStick.x * INT16_MAX, kpad_data.pro.rightStick.y * INT16_MAX);
      }
      else if (kpad_data.extensionType == WPAD_EXT_CLASSIC || kpad_data.extensionType == WPAD_EXT_MPLUS_CLASSIC) {
        uint32_t btns = kpad_data.classic.hold;
        int buttonFlags = 0;
#define CHECKBTN(v, f) if (btns & v) buttonFlags |= f;
        if (swap_buttons) {
          CHECKBTN(WPAD_CLASSIC_BUTTON_A,       B_FLAG);
          CHECKBTN(WPAD_CLASSIC_BUTTON_B,       A_FLAG);
          CHECKBTN(WPAD_CLASSIC_BUTTON_X,       Y_FLAG);
          CHECKBTN(WPAD_CLASSIC_BUTTON_Y,       X_FLAG);
        }
        else {
          CHECKBTN(WPAD_CLASSIC_BUTTON_A,       A_FLAG);
          CHECKBTN(WPAD_CLASSIC_BUTTON_B,       B_FLAG);
          CHECKBTN(WPAD_CLASSIC_BUTTON_X,       X_FLAG);
          CHECKBTN(WPAD_CLASSIC_BUTTON_Y,       Y_FLAG);
        }
        CHECKBTN(WPAD_CLASSIC_BUTTON_UP,      UP_FLAG);
        CHECKBTN(WPAD_CLASSIC_BUTTON_DOWN,    DOWN_FLAG);
        CHECKBTN(WPAD_CLASSIC_BUTTON_LEFT,    LEFT_FLAG);
        CHECKBTN(WPAD_CLASSIC_BUTTON_RIGHT,   RIGHT_FLAG);
        CHECKBTN(WPAD_CLASSIC_BUTTON_L,       LB_FLAG);
        CHECKBTN(WPAD_CLASSIC_BUTTON_R,       RB_FLAG);
        // don't have stick buttons on a classic controller
        // CHECKBTN(WPAD_CLASSIC_BUTTON_STICK_L, LS_CLK_FLAG);
        // CHECKBTN(WPAD_CLASSIC_BUTTON_STICK_R, RS_CLK_FLAG);
        CHECKBTN(WPAD_CLASSIC_BUTTON_PLUS,    PLAY_FLAG);
        CHECKBTN(WPAD_CLASSIC_BUTTON_MINUS,   BACK_FLAG);
        CHECKBTN(WPAD_CLASSIC_BUTTON_HOME,    SPECIAL_FLAG);
#undef CHECKBTN

        // If the button was just pressed, reset to current time
        if (kpad_data.classic.trigger & WPAD_CLASSIC_BUTTON_HOME)
          home_pressed[controllerNumber] = millis();

        if (btns & WPAD_CLASSIC_BUTTON_HOME && millis() - home_pressed[controllerNumber] > 3000) {
          state = STATE_STOP_STREAM;
          return;
        }

        LiSendMultiControllerEvent(controllerNumber++, gamepad_mask, buttonFlags,
          (kpad_data.classic.hold & WPAD_CLASSIC_BUTTON_ZL) ? 0xFF : 0x00,
          (kpad_data.classic.hold & WPAD_CLASSIC_BUTTON_ZR) ? 0xFF : 0x00,
          kpad_data.classic.leftStick.x * INT16_MAX, kpad_data.classic.leftStick.y * INT16_MAX,
          kpad_data.classic.rightStick.x * INT16_MAX, kpad_data.classic.rightStick.y * INT16_MAX);
      }
    }
  }
}

uint32_t wiiu_input_num_controllers(void)
{
  uint32_t numControllers = !disable_gamepad;

  WPADExtensionType type;
  for (int i = 0; i < 4; i++) {
    if (WPADProbe((WPADChan) i, &type) == 0) {
      if (type == WPAD_EXT_PRO_CONTROLLER || type == WPAD_EXT_CLASSIC || type == WPAD_EXT_MPLUS_CLASSIC) {
        numControllers++;
      }
    }
  }

  if (numControllers > 4) {
    numControllers = 4;
  }

  return numControllers;
}

uint32_t wiiu_input_buttons_triggered(void)
{
  uint32_t btns = 0;

  VPADStatus vpad;
  VPADReadError vpad_err;
  VPADRead(VPAD_CHAN_0, &vpad, 1, &vpad_err);
  if (vpad_err == VPAD_READ_SUCCESS) {
    btns |= vpad.trigger;
  }

  KPADStatus kpad_data = {0};
	int32_t kpad_err = -1;
	for (int i = 0; i < 4; i++) {
		KPADReadEx((KPADChan) i, &kpad_data, 1, &kpad_err);
		if (kpad_err == KPAD_ERROR_OK) {
      if (kpad_data.extensionType == WPAD_EXT_PRO_CONTROLLER) {
#define MAPBTNS(b, v) if (kpad_data.pro.trigger & b) btns |= v;
        MAPBTNS(WPAD_PRO_BUTTON_UP,       VPAD_BUTTON_UP);
        MAPBTNS(WPAD_PRO_BUTTON_LEFT,     VPAD_BUTTON_LEFT);
        MAPBTNS(WPAD_PRO_TRIGGER_ZR,      VPAD_BUTTON_ZR);
        MAPBTNS(WPAD_PRO_BUTTON_X,        VPAD_BUTTON_X);
        MAPBTNS(WPAD_PRO_BUTTON_A,        VPAD_BUTTON_A);
        MAPBTNS(WPAD_PRO_BUTTON_Y,        VPAD_BUTTON_Y);
        MAPBTNS(WPAD_PRO_BUTTON_B,        VPAD_BUTTON_B);
        MAPBTNS(WPAD_PRO_TRIGGER_ZL,      VPAD_BUTTON_ZL);
        MAPBTNS(WPAD_PRO_TRIGGER_R,       VPAD_BUTTON_R);
        MAPBTNS(WPAD_PRO_BUTTON_PLUS,     VPAD_BUTTON_PLUS);
        MAPBTNS(WPAD_PRO_BUTTON_HOME,     VPAD_BUTTON_HOME);
        MAPBTNS(WPAD_PRO_BUTTON_MINUS,    VPAD_BUTTON_MINUS);
        MAPBTNS(WPAD_PRO_TRIGGER_L,       VPAD_BUTTON_L);
        MAPBTNS(WPAD_PRO_BUTTON_DOWN,     VPAD_BUTTON_DOWN);
        MAPBTNS(WPAD_PRO_BUTTON_RIGHT,    VPAD_BUTTON_RIGHT);
        MAPBTNS(WPAD_PRO_BUTTON_STICK_R,  VPAD_BUTTON_STICK_R);
        MAPBTNS(WPAD_PRO_BUTTON_STICK_L,  VPAD_BUTTON_STICK_L);
#undef MAPBTNS
      }
      else if (kpad_data.extensionType == WPAD_EXT_CLASSIC || kpad_data.extensionType == WPAD_EXT_MPLUS_CLASSIC) {
#define MAPBTNS(b, v) if (kpad_data.classic.trigger & b) btns |= v;
        MAPBTNS(WPAD_CLASSIC_BUTTON_UP,     VPAD_BUTTON_UP);
        MAPBTNS(WPAD_CLASSIC_BUTTON_LEFT,   VPAD_BUTTON_LEFT);
        MAPBTNS(WPAD_CLASSIC_BUTTON_ZR,     VPAD_BUTTON_ZR);
        MAPBTNS(WPAD_CLASSIC_BUTTON_X,      VPAD_BUTTON_X);
        MAPBTNS(WPAD_CLASSIC_BUTTON_A,      VPAD_BUTTON_A);
        MAPBTNS(WPAD_CLASSIC_BUTTON_Y,      VPAD_BUTTON_Y);
        MAPBTNS(WPAD_CLASSIC_BUTTON_B,      VPAD_BUTTON_B);
        MAPBTNS(WPAD_CLASSIC_BUTTON_ZL,     VPAD_BUTTON_ZL);
        MAPBTNS(WPAD_CLASSIC_BUTTON_R,      VPAD_BUTTON_R);
        MAPBTNS(WPAD_CLASSIC_BUTTON_PLUS,   VPAD_BUTTON_PLUS);
        MAPBTNS(WPAD_CLASSIC_BUTTON_HOME,   VPAD_BUTTON_HOME);
        MAPBTNS(WPAD_CLASSIC_BUTTON_MINUS,  VPAD_BUTTON_MINUS);
        MAPBTNS(WPAD_CLASSIC_BUTTON_L,      VPAD_BUTTON_L);
        MAPBTNS(WPAD_CLASSIC_BUTTON_DOWN,   VPAD_BUTTON_DOWN);
        MAPBTNS(WPAD_CLASSIC_BUTTON_RIGHT,  VPAD_BUTTON_RIGHT);
#undef MAPBTNS
      }
      else {
        // meh we can't really map a wiimote to gamepad
      }
    }
  }

  return btns;
}

static void alarm_callback(OSAlarm* alarm, OSContext* ctx)
{
  wiiu_input_update();
}

static int input_thread_proc(int argc, const char **argv)
{
  OSCreateAlarm(&inputAlarm);
  OSSetPeriodicAlarm(&inputAlarm, 0, OSMillisecondsToTicks(input_update_rate), alarm_callback);

  while (thread_running) {
    OSWaitAlarm(&inputAlarm);
  }
}

static void thread_deallocator(OSThread *thread, void *stack)
{
  free(stack);
}

void start_input_thread(void)
{
  
  if (vban_enable || mic_button_key != 0) {
    initializeMic();
    WHBLogPrintf("Initialized microphone");
  }

  if (vban_enable) { 
    init_vban_sender(vban_ipaddress, vban_port, "WiiUMic", 1);
    WHBLogPrintf("Initialized VBAN sender");
  }
  const int stack_size = 4 * 1024 * 1024;
  uint8_t* stack = (uint8_t*)memalign(16, stack_size);
  if (!stack) {
    return;
  }

  if (!OSCreateThread(&inputThread,
                      input_thread_proc,
                      0, NULL,
                      stack + stack_size, stack_size,
                      0x10, OS_THREAD_ATTRIB_AFFINITY_ANY))
  {
    free(stack);
    return;
  }

  thread_running = 1;

  OSSetThreadName(&inputThread, "PadInput");
  OSSetThreadDeallocator(&inputThread, thread_deallocator);
  OSResumeThread(&inputThread);
}

void stop_input_thread(void)
{
  thread_running = 0;

  OSCancelAlarm(&inputAlarm);
  OSJoinThread(&inputThread, NULL);

  if (vban_enable || mic_button_key != 0) {
    MICClose(micHandle);
    MICUninit(micHandle);
  }

  if (vban_enable && udp_socket >= 0) {
    close(udp_socket);
    udp_socket = -1;
  }
}
