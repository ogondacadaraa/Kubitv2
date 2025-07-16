// This file should be located at: Kubit-PlatformIO-Project/src/main.cpp
// Based on working idf-wav-sdcard example

#include <Arduino.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Suppress legacy driver deprecation warnings from third-party libraries
// NOTE: The I2S and ADC libraries used in this project rely on legacy ESP32 drivers
// which are deprecated. The warnings have been suppressed until the libraries are
// updated to use the newer esp_adc/adc_oneshot.h and driver/i2s_std.h APIs.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include "driver/i2s.h"
#include "driver/adc.h"
#pragma GCC diagnostic pop

#include <math.h>

// Include project libraries from the 'lib' folder
#include "I2SOutput.h"
#include "I2SMEMSSampler.h"
#include "SDCard.h"
#include "SPIFFS.h"
#include "WAVFileReader.h"
#include "WAVFileWriter.h"
#include "esp_timer.h"
#include "GyroscopeControl.h"
#include "EmotionSystem.h"
#include "Output.h"
#include "config.h"

static const char *TAG = "app";

// SD card pin definitions (only what we need)
#define PIN_NUM_MISO GPIO_NUM_4
#define PIN_NUM_CLK GPIO_NUM_14
#define PIN_NUM_MOSI GPIO_NUM_15
#define PIN_NUM_CS GPIO_NUM_25

// I2S Speaker pin definitions (for audio playback)
#define I2S_SPEAKER_SERIAL_CLOCK GPIO_NUM_19
#define I2S_SPEAKER_LEFT_RIGHT_CLOCK GPIO_NUM_27
#define I2S_SPEAKER_SERIAL_DATA GPIO_NUM_18

// TTP229 Touch Keypad Pins
#define SCLPin      33  // Clock pin for TTP229
#define SDAPin      32  // Data pin for TTP229

// Button mapping for TTP229 (using buttons 1-8)
#define TOUCH_BUTTON_FORWARD      1
#define TOUCH_BUTTON_BACKWARD     2
#define TOUCH_BUTTON_LEFT         3
#define TOUCH_BUTTON_RIGHT        4
#define TOUCH_BUTTON_DELETE       5
#define TOUCH_BUTTON_RECORD_VOICE 6
#define TOUCH_BUTTON_START_STOP   7
#define TOUCH_BUTTON_HEAD         8

// L298N Motor Driver Pins
const int MOTOR_IN1 = 23;
const int MOTOR_IN2 = 2;
const int MOTOR_IN3 = 12;
const int MOTOR_IN4 = 13;

// Constants
const int MAX_COMMANDS = 50;
const int MOVETIME = 510;
const int TURNTIME = 620;
const int VIBRATE_TIME = 200;
const int VIBRATE_INTERVAL = 50;

// Command Codes
const int CMD_FORWARD = 1;
const int CMD_BACKWARD = 2;
const int CMD_LEFT = 3;
const int CMD_RIGHT = 4;
const int CMD_PLAY_RECORDING = 5;
const int CMD_TURN_90_LEFT = 6;
const int CMD_TURN_90_RIGHT = 7;

// Global Variables
int commandBuffer[MAX_COMMANDS];
int commandCount = 0;
int currentCommandIndex = 0;
bool executing = false;
bool paused = false;
bool sound_status = true;
int sound_level = 80;
bool isRecording = false;
bool hasRecording = false;
bool microphoneActive = false; // Track microphone state
unsigned long startStopButtonPressTime = 0;
bool isVibrating = false;

// Multitouch and Emotion System Variables
uint16_t currentTouchState = 0; // Current buttons pressed (bitmask)
uint16_t lastTouchState = 0;    // Previous touch state
unsigned long touchStartTime = 0;
unsigned long touchHoldTime = 500; // Minimum hold time for multitouch (ms)
bool multitouchActive = false;
bool emotionPlaying = false;

// Audio control variables
TaskHandle_t audioTaskHandle = NULL;
const char* audioFileToPlay = NULL;
volatile bool isPlayingAudio = false;


// Component objects
GyroscopeControl *gyro = nullptr;
EmotionSystem *emotionSystem = nullptr;
I2SOutput *output = nullptr;
I2SMEMSSampler *input = nullptr;

// Audio control variables - REMOVED DUPLICATE DECLARATION
WAVFileReader *currentWavReader = nullptr;
FILE *currentAudioFile = nullptr;


// TTP229 Touch Button Event Structure
struct CTtp229ButtonEvent {
  uint8_t ButtonNumber : 5;
  uint8_t IsButtonReleased : 1;
};

// TTP229 Touch Button Class
class CTtP229TouchButton {
private:
    struct CTtp229Prop {
      uint16_t SclPin : 6;
      uint16_t SdoPin : 6;
      uint16_t Is16Button : 1;
      uint16_t PreviousButtonValue;
    };
    static CTtp229Prop g_prop;
    static uint8_t GetButtonNumberFromFlag(uint16_t buttonsChanged);
public:
    static void Configure(int sclPin, int sdoPin, bool is16Button = true);
    static CTtp229ButtonEvent GetButtonEvent();
    static uint16_t GetPressedButton(); // Make this public for multitouch
};

// Forward Declarations
void moveForward();
void moveBackward();
void turnLeft();
void turnRight();
void stopMotors();
void playAudio(const char *fname);
void audioPlaybackTask(void *parameter);
void recordAudio();
void recordingTask(void *parameter);
void readTouchButtons();
void handleButtonPress(uint8_t buttonNumber);
void handleButtonRelease(uint8_t buttonNumber);
void addCommand(int cmd, const char* name);
void executeCommands();
void executeSingleCommand(int command);
void performHeadVibration();
void handleMultitouch(uint16_t touchMask);
void playEmotion(const char* emotionName);
bool checkEmotionCombination(uint16_t touchMask);
uint8_t countBitsSet(uint16_t value);


// TTP229 Implementation
CTtP229TouchButton::CTtp229Prop CTtP229TouchButton::g_prop;
uint16_t CTtP229TouchButton::GetPressedButton() {
    uint16_t buttonsPressed = 0;
    uint8_t maxCnt = g_prop.Is16Button ? 16 : 8;
    for (uint8_t ndx = 0; ndx < maxCnt; ndx++) {
        digitalWrite(g_prop.SclPin, LOW);
        delayMicroseconds(1);
        digitalWrite(g_prop.SclPin, HIGH);
        int val = digitalRead(g_prop.SdoPin);
        delayMicroseconds(1);
        if (LOW == val) {
            buttonsPressed |= (1 << ndx);
        }
    }
    
    // INVERT the logic: TTP229 reads LOW when pressed, HIGH when not pressed
    // So we need to invert the bits to get the actual pressed buttons
    uint16_t maxMask = (1 << maxCnt) - 1; // Create mask for valid bits (0xFF for 8 buttons, 0xFFFF for 16)
    buttonsPressed = (~buttonsPressed) & maxMask; // Invert and mask to valid range
    
    return buttonsPressed;
}
uint8_t CTtP229TouchButton::GetButtonNumberFromFlag(uint16_t buttonsChanged) {
    uint16_t flag = 1;
    for (uint8_t ndx = 1; ndx <= 16; ndx++, flag <<= 1) {
        if ((buttonsChanged & flag) != 0) {
            if ((buttonsChanged & ~flag) != 0) {
                ndx |= 0x80; // More changes present
            }
            return ndx;
        }
    }
    return 0;
}
void CTtP229TouchButton::Configure(int sclPin, int sdoPin, bool is16Button) {
    g_prop.SclPin = sclPin;
    g_prop.SdoPin = sdoPin;
    g_prop.Is16Button = false; // Force 8-button mode for stability
    g_prop.PreviousButtonValue = 0;
    pinMode(sclPin, OUTPUT);
    digitalWrite(sclPin, HIGH);
    pinMode(sdoPin, INPUT);
    Serial.printf("TTP229 Configured - SCL: %d, SDO: %d, Keys: %d (FORCED 8-button mode)\n", sclPin, sdoPin, 8);
}
CTtp229ButtonEvent CTtP229TouchButton::GetButtonEvent() {
    CTtp229ButtonEvent returnValue = {0, 0};
    uint16_t currValue = GetPressedButton();
    
    uint16_t changes = g_prop.PreviousButtonValue ^ currValue;
    uint16_t pressed = (changes & currValue);
    uint16_t released = (changes & g_prop.PreviousButtonValue);
    if (0 != pressed) {
        uint8_t buttonNumber = GetButtonNumberFromFlag(pressed);
        returnValue.ButtonNumber = (buttonNumber & 0x7F);
        uint16_t mask = (1 << (returnValue.ButtonNumber - 1));
        g_prop.PreviousButtonValue |= mask;
    } else if (0 != released) {
        uint8_t buttonNumber = GetButtonNumberFromFlag(released);
        returnValue.ButtonNumber = (buttonNumber & 0x7F);
        returnValue.IsButtonReleased = true;
        g_prop.PreviousButtonValue &= ~(1 << (returnValue.ButtonNumber - 1));
    }
    return returnValue;
}

// Create global instance of TTP229 button handler
CTtP229TouchButton g_ttp229Button;

// ========================================================================
//                          FUNCTION IMPLEMENTATIONS
// ========================================================================

// Audio recording function - Simple like idf-wav-sdcard
void recordAudio() {
  if (!input) {
    Serial.println("No microphone available for recording");
    return;
  }
  
  if (isRecording) {
    // Stop recording if already recording
    isRecording = false;
    Serial.println("⏹️ Recording stopped by button press");
    return;
  }

  // Start recording in a separate task (non-blocking)
  Serial.println("🎤 Starting recording task...");
  xTaskCreate(recordingTask, "Recording", 8192, NULL, 1, NULL);
}

// Non-blocking recording task
void recordingTask(void *parameter) {
  if (!input) {
    Serial.println("No microphone in recording task");
    vTaskDelete(NULL);
    return;
  }
  
  // Stop any playing audio first to free I2S resources
  if (isPlayingAudio) {
    isPlayingAudio = false; // Signal to stop first
    playAudio(nullptr); // Correctly signal the audio task to stop
    vTaskDelay(pdMS_TO_TICKS(500)); // Give it more time to clean up
  }

  int16_t *samples = (int16_t *)malloc(sizeof(int16_t) * 1024);
  if (!samples) {
    Serial.println("Failed to allocate memory for recording");
    vTaskDelete(NULL);
    return;
  }

  ESP_LOGI(TAG, "Start recording to /sdcard/user_recording.wav");
  
  // Start microphone input
  input->start();
  microphoneActive = true; // Track state
  vTaskDelay(pdMS_TO_TICKS(200)); // Give I2S more time to stabilize
  
  // Always overwrite the existing recording file
  FILE *fp = fopen("/sdcard/user_recording.wav", "wb");
  if (!fp) {
    Serial.println("ERROR: Failed to open recording file on SD card!");
    input->stop();
    free(samples);
    vTaskDelete(NULL);
    return;
  }

  // Create a new wave file writer - use 16kHz sample rate to match microphone
  WAVFileWriter *writer = new WAVFileWriter(fp, 16000); // Fixed sample rate
  if (!writer) {
    Serial.println("Failed to create WAV writer");
    fclose(fp); // Make sure to close file on error
    input->stop();
    free(samples);
    vTaskDelete(NULL);
    return;
  }

  isRecording = true;
  int totalSamples = 0;
  
  Serial.println("🔴 Recording... Press RECORD button again to stop");
  
  // Record until stopped by button press - this runs in background!
  int loopCount = 0;
  while (isRecording) {
    int samples_read = input->read(samples, 1024);
    loopCount++;
    
    if (loopCount % 20 == 0) { // Reduce logging frequency
      Serial.printf("Loop %d: Read %d samples from microphone\n", loopCount, samples_read);
    }
    
    if (samples_read > 0) {
      // Check if the samples are not all zeros and show some raw values
      bool all_zeros = true;
      int16_t max_sample = 0;
      int16_t min_sample = 0;
      int non_zero_count = 0;
      
      for (int i = 0; i < samples_read; i++) {
        if (samples[i] != 0) {
          all_zeros = false;
          non_zero_count++;
          if (abs(samples[i]) > abs(max_sample)) {
            max_sample = samples[i];
          }
          if (samples[i] < min_sample) {
            min_sample = samples[i];
          }
        }
      }
      
      if (all_zeros) {
        if (loopCount % 40 == 0) { // Print every 40th iteration for zero samples
          ESP_LOGW(TAG, "Loop %d: Read %d samples, but all are zero! Check microphone connection.", loopCount, samples_read);
        }
      } else {
        if (loopCount % 20 == 0) { // Reduce logging frequency
          Serial.printf("Loop %d: Writing %d samples (non-zero: %d, max: %d, min: %d) to file\n", 
                       loopCount, samples_read, non_zero_count, max_sample, min_sample);
        }
        writer->write(samples, samples_read);
        totalSamples += samples_read;
      }
    } else {
      if (loopCount % 40 == 0) {
        Serial.printf("Loop %d: No samples read from microphone\n", loopCount);
      }
    }
    
    // Small delay to prevent overwhelming the system
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  
  // Stop recording - CRITICAL: Proper cleanup order
  input->stop();
  microphoneActive = false; // Update state
  
  // Ensure all data is written and file is properly closed
  if (writer) {
    writer->finish();
    delete writer; // Delete writer first
    writer = nullptr;
  }
  
  if (fp) {
    fclose(fp); // Then close file
    fp = nullptr;
  }
  
  free(samples);
  
  if (totalSamples > 0) {
    hasRecording = true;
    ESP_LOGI(TAG, "Recording complete: %d samples recorded", totalSamples);
    Serial.printf("✅ Recording saved to /sdcard/user_recording.wav (%d samples)\n", totalSamples);
    
    // Small delay before playing confirmation
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Play confirmation sound
    playAudio("/sdcard/record_off.wav");
    
    // Wait for confirmation sound to finish, then automatically add the play command
    while(isPlayingAudio) {
      vTaskDelay(pdMS_TO_TICKS(50));
    }
    addCommand(CMD_PLAY_RECORDING, "Play Recording");
  } else {
    hasRecording = false;
    ESP_LOGE(TAG, "Recording failed: no samples recorded");
    Serial.println("❌ Recording failed - no audio data captured");
  }
  
  // Clean up task
  vTaskDelete(NULL);
}

// Non-blocking audio playback task
void audioPlaybackTask(void *parameter) {
  const char *fname = (const char *)parameter;
  
  // Don't play if sound is disabled
  if (!output || !sound_status) {
    ESP_LOGW(TAG, "Audio task aborted: sound disabled or output not initialized.");
    audioTaskHandle = NULL;
    vTaskDelete(NULL);
    return;
  }

  // Stop microphone input to free I2S resources during playback - but only if it's running
  if (input && microphoneActive) {
    input->stop();
    microphoneActive = false;
    Serial.println("Stopped microphone for audio playback");
    vTaskDelay(pdMS_TO_TICKS(200)); // Give I2S more time to properly release
  }

  FILE *audioFile = fopen(fname, "rb");
  if (!audioFile) {
    ESP_LOGE(TAG, "Failed to open audio file: %s", fname);
    audioTaskHandle = NULL;
    vTaskDelete(NULL);
    return;
  }

  WAVFileReader *wavReader = new WAVFileReader(audioFile);
  if (!wavReader) {
    ESP_LOGE(TAG, "Failed to create WAV reader for: %s", fname);
    fclose(audioFile);
    audioTaskHandle = NULL;
    vTaskDelete(NULL);
    return;
  }

  ESP_LOGI(TAG, "🔊 Playing audio task started: %s", fname);
  Serial.printf("WAV file info - Sample rate: %d\n", wavReader->sample_rate());
  
  // This flag is now controlled externally to interrupt playback
  isPlayingAudio = true; 
  
  // Start output with the file's sample rate
  output->start(wavReader->sample_rate());

  int16_t *samples = (int16_t *)malloc(sizeof(int16_t) * 1024);
  if (!samples) {
    ESP_LOGE(TAG, "Failed to allocate audio buffer for playback");
    delete wavReader;
    fclose(audioFile);
    output->stop();
    isPlayingAudio = false;
    audioTaskHandle = NULL;
    vTaskDelete(NULL);
    return;
  }

  int totalSamplesPlayed = 0;
  int loopCount = 0;
  
  // Loop continues as long as isPlayingAudio is true and there's data
  while (isPlayingAudio) {
    int samples_read = wavReader->read(samples, 1024);
    if (samples_read == 0) {
      break; // End of file
    }
    
    loopCount++;
    if (loopCount % 100 == 0) { // Reduce logging frequency
      Serial.printf("Playback loop %d: Playing %d samples\n", loopCount, samples_read);
    }
    
    // Adjust volume
    for (int i = 0; i < samples_read; i++) {
      samples[i] = (samples[i] * sound_level) / 100;
    }
    
    output->write(samples, samples_read);
    totalSamplesPlayed += samples_read;
    
    // Small delay to prevent overwhelming the system
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  // Cleanup - CRITICAL: Proper cleanup order
  free(samples);
  delete wavReader; // Delete reader first
  fclose(audioFile); // Then close file
  output->stop(); // Finally stop I2S output
  
  if (isPlayingAudio) {
      ESP_LOGI(TAG, "✅ Finished playing audio: %s (%d samples)", fname, totalSamplesPlayed);
  } else {
      ESP_LOGI(TAG, "🛑 Audio interrupted: %s", fname);
  }

  // Signal that playback is complete
  isPlayingAudio = false;
  audioTaskHandle = NULL;
  vTaskDelete(NULL); // Task is done, deletes itself
}


// Proper audio playback function - Non-blocking and interruptible
void playAudio(const char *fname) {
  // If a sound is already playing, signal it to stop and wait for cleanup.
  if (audioTaskHandle != NULL) {
    isPlayingAudio = false; // Signal the existing task to stop
    
    // Wait for the task to finish properly
    int timeout = 0;
    while (audioTaskHandle != NULL && timeout < 100) { // Wait up to 1 second
      vTaskDelay(pdMS_TO_TICKS(10));
      timeout++;
    }
    
    // If task is still running, force cleanup
    if (audioTaskHandle != NULL) {
      Serial.println("Force terminating audio task");
      vTaskDelete(audioTaskHandle);
      audioTaskHandle = NULL;
      isPlayingAudio = false;
    }
  }
  
  // If fname is null, we just wanted to stop the audio, not play a new one.
  if (fname == nullptr) {
      return;
  }

  audioFileToPlay = fname;
  
  // Create a new task for audio playback with larger stack
  xTaskCreate(
    audioPlaybackTask,
    "AudioPlayback",
    8192, // Increased stack size
    (void*)audioFileToPlay,
    5, // Priority
    &audioTaskHandle
  );
}

// ====================================================================================
//                                  SETUP FUNCTION
// ====================================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n\n=== Booting Kubit Robot (Simple Audio) ===");

  // Create component objects
  ESP_LOGI(TAG, "Creating component objects");
  
  ESP_LOGI(TAG, "Creating gyroscope");
  gyro = new GyroscopeControl();
  
  ESP_LOGI(TAG, "Creating emotion system");
  emotionSystem = new EmotionSystem();

  // Initialize File System
  ESP_LOGI(TAG, "Initializing File System");
  // Force SD card usage - SPIFFS is disabled in config.h
  Serial.println("=== FORCING SD CARD USAGE (SPIFFS DISABLED) ===");
  SDCard *sdcard = new SDCard("/sdcard", PIN_NUM_MISO, PIN_NUM_MOSI, PIN_NUM_CLK, PIN_NUM_CS);
  if (sdcard) {
    Serial.println("✓ SD Card initialized successfully at /sdcard");
    
    // Test file access to confirm SD card is working
    FILE *testFile = fopen("/sdcard/test.txt", "w");
    if (testFile) {
      fprintf(testFile, "SD card test\n");
      fclose(testFile);
      Serial.println("✓ SD card write test successful");
      
      // Clean up test file
      remove("/sdcard/test.txt");
    } else {
      Serial.println("✗ SD card write test failed");
    }
  } else {
    Serial.println("✗ SD Card initialization FAILED - audio features will not work");
  }

  // Initialize Touch Keypad
  ESP_LOGI(TAG, "Initializing Touch Keypad");
  g_ttp229Button.Configure(SCLPin, SDAPin, false); // Force 8-button mode

  // Initialize Motor Controller
  ESP_LOGI(TAG, "Initializing Motor Controller");
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT);
  pinMode(MOTOR_IN4, OUTPUT);
  stopMotors();

  // Initialize Gyroscope
  ESP_LOGI(TAG, "Initializing Gyroscope");
  if (gyro->initialize()) {
    Serial.println("Gyroscope initialized successfully");
  } else {
    Serial.println("Failed to initialize gyroscope");
  }

  // Initialize Audio Output
  Serial.println("Initializing Audio Output");
  
  // Create I2S speaker pin configuration
  i2s_pin_config_t i2s_speaker_pins = {
    .bck_io_num = I2S_SPEAKER_SERIAL_CLOCK,
    .ws_io_num = I2S_SPEAKER_LEFT_RIGHT_CLOCK,
    .data_out_num = I2S_SPEAKER_SERIAL_DATA,
    .data_in_num = I2S_PIN_NO_CHANGE
  };
  
  // Initialize output (speaker) - Use I2S_NUM_1 to avoid conflict with microphone
  output = new I2SOutput(I2S_NUM_1, i2s_speaker_pins);
  if (output) {
    Serial.println("✓ Audio output created successfully on I2S_NUM_1");
  } else {
    Serial.println("✗ Failed to create audio output");
  }
  
  // Initialize I2S Microphone for recording
  Serial.println("Initializing I2S Microphone for recording");
  i2s_pin_config_t i2s_mic_pins = {
    .bck_io_num = GPIO_NUM_26,
    .ws_io_num = GPIO_NUM_22,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = GPIO_NUM_21
  };
  
  // Create I2S config for microphone
  i2s_config_t i2s_mic_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT, // Change to RIGHT channel like your example
    .communication_format = I2S_COMM_FORMAT_STAND_I2S, // Use STAND_I2S like your example
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 1024,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  
  Serial.println("Creating I2S microphone...");
  input = new I2SMEMSSampler(I2S_NUM_0, i2s_mic_pins, i2s_mic_config, true); // Enable SPH0645 fix like your example
  if (input) {
    Serial.println("✓ I2S Microphone created successfully on I2S_NUM_0");
    Serial.print("I2S Microphone pins - BCK: "); Serial.print(GPIO_NUM_26);
    Serial.print(", WS: "); Serial.print(GPIO_NUM_22);
    Serial.print(", DATA: "); Serial.println(GPIO_NUM_21);
  } else {
    Serial.println("✗ Failed to create I2S microphone");
  }
  
  Serial.println("\n=== System Initialized ===");
}

// ====================================================================================
//                                     MAIN LOOP
// ====================================================================================
void loop() {
  if (!gyro || !emotionSystem) {
    Serial.println("FATAL: A component failed to initialize. Halting.");
    delay(1000);
    return;
  }
  
  gyro->update();
  
  if (emotionSystem->isEmotionActive()) {
    emotionSystem->updateEmotion(moveForward, moveBackward, turnLeft, turnRight, stopMotors, 
                                [](const char* fname){ playAudio(fname); });
  }
  
  readTouchButtons();
  
  // The main loop now only needs to call executeCommands.
  // The waiting and non-blocking logic is handled inside it.
  if (executing && !paused) {
    executeCommands();
  }
  
  // Reduced delay for better responsiveness
  delay(10); // Use a small delay instead of microseconds
}

// ====================================================================================
//                             Function Implementations
// ====================================================================================

void readTouchButtons() {
  // Get raw button state directly from TTP229 (now properly inverted)
  uint16_t rawButtons = CTtP229TouchButton::GetPressedButton();
  
  currentTouchState = rawButtons;
  
  // Count how many buttons are currently pressed
  uint8_t buttonsPressed = countBitsSet(currentTouchState);
  
  // Debug output when touch state changes
  static uint16_t lastDebugState = 0;
  if (currentTouchState != lastDebugState) {
    Serial.printf("🎯 TOUCH STATE: %d buttons pressed, mask=0x%02X (", buttonsPressed, currentTouchState);
    for (int i = 0; i < 8; i++) {
      if (currentTouchState & (1 << i)) {
        Serial.printf("%d ", i + 1);
      }
    }
    Serial.println(")");
    lastDebugState = currentTouchState;
  }
  
  // Allow any number of buttons (removed max limit)
  // Serial.printf("🎯 Detected %d buttons pressed\n", buttonsPressed);
  
  // Handle multitouch (2 or more buttons)
  if (buttonsPressed >= 2) {
    if (!multitouchActive) {
      // Start of multitouch
      multitouchActive = true;
      touchStartTime = millis();
      Serial.printf("🖐️ MULTITOUCH START: %d buttons, mask=0x%02X\n", buttonsPressed, currentTouchState);
    } else if (currentTouchState != lastTouchState) {
      // Touch pattern changed, restart timer
      touchStartTime = millis();
      Serial.printf("🖐️ MULTITOUCH CHANGED: %d buttons, mask=0x%02X\n", buttonsPressed, currentTouchState);
    } else {
      // Check if multitouch has been held long enough and is stable
      if (millis() - touchStartTime >= touchHoldTime) {
        // Execute multitouch command
        handleMultitouch(currentTouchState);
        multitouchActive = false; // Reset to prevent repeated execution
        
        // Add delay to prevent immediate re-triggering
        vTaskDelay(pdMS_TO_TICKS(1000));
      }
    }
  } 
  // Handle single touch (1 button)
  else if (buttonsPressed == 1) {
    if (multitouchActive) {
      // End multitouch without executing if it was too short
      multitouchActive = false;
      Serial.println("🖐️ MULTITOUCH CANCELLED (changed to single touch)");
    }
    
    // Handle single button press/release using direct bit detection
    // Find which button is pressed
    for (int i = 0; i < 8; i++) {
      if (currentTouchState & (1 << i)) {
        uint8_t buttonNumber = i + 1; // Convert to 1-based button numbering
        
        // Check if this is a new press (wasn't pressed before)
        if (!(lastTouchState & (1 << i))) {
          Serial.printf("🔘 SINGLE BUTTON PRESS: %d\n", buttonNumber);
          handleButtonPress(buttonNumber);
        }
        break; // Only process one button for single touch
      }
    }
    
    // Handle button releases 
    for (int i = 0; i < 8; i++) {
      if ((lastTouchState & (1 << i)) && !(currentTouchState & (1 << i))) {
        uint8_t buttonNumber = i + 1; // Convert to 1-based button numbering
        Serial.printf("🔘 SINGLE BUTTON RELEASE: %d\n", buttonNumber);
        handleButtonRelease(buttonNumber);
        break; // Only process one button release at a time
      }
    }
  }
  // Handle no touch (0 buttons)
  else {
    if (multitouchActive) {
      // End multitouch without executing
      multitouchActive = false;
      Serial.println("🖐️ MULTITOUCH ENDED (no buttons)");
    }
    
    // Check for button releases using direct bit detection
    for (int i = 0; i < 8; i++) {
      if ((lastTouchState & (1 << i)) && !(currentTouchState & (1 << i))) {
        uint8_t buttonNumber = i + 1; // Convert to 1-based button numbering
        Serial.printf("🔘 BUTTON RELEASE (no touch): %d\n", buttonNumber);
        handleButtonRelease(buttonNumber);
      }
    }
  }
  
  lastTouchState = currentTouchState;
}

void handleButtonPress(uint8_t buttonNumber) {
    if (!emotionSystem) return;

    // Don't interrupt emotions with single button presses
    if (emotionPlaying) {
        Serial.println("🎭 Emotion playing, ignoring single button press");
        return;
    }

    // CRITICAL: Don't allow audio operations while recording is active to prevent crashes
    // EXCEPT for the record button which should be able to stop recording
    if (isRecording && buttonNumber != TOUCH_BUTTON_RECORD_VOICE) {
        Serial.printf("🎤 Recording active - button %d press ignored to prevent I2S conflict\n", buttonNumber);
        return;
    }

    // First, check if this button press completes an emotion sequence.
    bool emotionTriggered = emotionSystem->addButtonToSequence(buttonNumber);

    // If an emotion was just triggered, let it play without interruption.
    if (emotionTriggered) {
        Serial.println("🤖 Emotion triggered! Allowing emotion to take control.");
        return; // IMPORTANT: Exit now to prevent the new emotion from being stopped.
    }

    // If NO emotion was triggered, we can proceed with normal button logic.
    // ALWAYS stop any currently playing sound for immediate audio feedback
    if (isPlayingAudio) {
        playAudio(nullptr); // Stop audio gracefully to allow new sound
    }
    if (emotionSystem->isEmotionActive()) {
        emotionSystem->forceStopEmotion();
    }

    // Long press on START/STOP is handled in handleButtonRelease
    if (buttonNumber == TOUCH_BUTTON_START_STOP) {
        startStopButtonPressTime = millis();
        return;
    }
    
    // If we are executing, any button press (except START/STOP) will stop it
    if (executing) {
        executing = false;
        paused = false;
        currentCommandIndex = 0;
        stopMotors();
        Serial.println("⏹️ Execution stopped by user action.");
        playAudio("/sdcard/stop.wav");
        // We still process the button press below
    }
    
    switch (buttonNumber) {
        case TOUCH_BUTTON_FORWARD:      
            Serial.println("🔊 Forward button feedback + command");
            playAudio("/sdcard/forward.wav");
            addCommand(CMD_FORWARD, "Forward"); 
            break;
        case TOUCH_BUTTON_BACKWARD:     
            Serial.println("🔊 Backward button feedback + command");
            playAudio("/sdcard/backward.wav");
            addCommand(CMD_BACKWARD, "Backward"); 
            break;
        case TOUCH_BUTTON_LEFT:         
            Serial.println("🔊 Left button feedback + command");
            playAudio("/sdcard/left.wav");
            addCommand(CMD_TURN_90_LEFT, "Turn 90 Left"); 
            break;
        case TOUCH_BUTTON_RIGHT:        
            Serial.println("🔊 Right button feedback + command");
            playAudio("/sdcard/right.wav");
            addCommand(CMD_TURN_90_RIGHT, "Turn 90 Right"); 
            break;
        case TOUCH_BUTTON_HEAD:         
            Serial.println("🤖 Head button feedback + vibration");
            playAudio("/sdcard/head_touch.wav");
            performHeadVibration(); 
            break;
        
        case TOUCH_BUTTON_DELETE:
            Serial.println("🗑️ Delete button feedback + clear");
            playAudio("/sdcard/delete.wav");
            commandCount = 0;
            currentCommandIndex = 0;
            executing = false;
            paused = false;
            hasRecording = false; // Also clear the recording flag
            Serial.println("All commands and recording deleted");
            break;
            
        case TOUCH_BUTTON_RECORD_VOICE:
            if (isRecording) {
                // This signals the recording task to stop
                isRecording = false; 
                Serial.println("⏹️ Recording stopped by button press");
                // The recording task will play the "off" sound and add the command
            } else {
                // Start recording - but only if no audio is playing
                if (isPlayingAudio) {
                    Serial.println("🎤 Cannot start recording while audio is playing");
                    return;
                }
                playAudio("/sdcard/record_on.wav");
                recordAudio();
            }
            break;
    }
}

void handleButtonRelease(uint8_t buttonNumber) {
    if (buttonNumber == TOUCH_BUTTON_START_STOP) {
        // Check if it was a short press (less than 1 second)
        if (millis() - startStopButtonPressTime < 1000) {
            if (!executing) {
                if (commandCount > 0) {
                    Serial.println("▶️ Starting execution...");
                    playAudio("/sdcard/start.wav");
                    // Wait for the start sound to finish before executing commands
                    while(isPlayingAudio) {
                        vTaskDelay(pdMS_TO_TICKS(20));
                    }
                    executing = true;
                    paused = false;
                    currentCommandIndex = 0;
                } else {
                    Serial.println("No commands to execute.");
                }
            } else {
                // Toggle pause state
                paused = !paused;
                if (paused) {
                    Serial.println("⏸️ Execution paused.");
                    playAudio("/sdcard/pause.wav");
                } else {
                    Serial.println("⏯️ Execution resumed.");
                    playAudio("/sdcard/resume.wav");
                }
            }
        }
        // Reset the timer
        startStopButtonPressTime = 0;
    }
}

void addCommand(int cmd, const char* name) {
    if (commandCount < MAX_COMMANDS) {
        commandBuffer[commandCount++] = cmd;
        Serial.printf("Added command: %s. Total commands: %d\n", name, commandCount);
    } else {
        Serial.println("Command buffer is full!");
    }
}

void executeCommands() {
  // This function is now non-blocking. It executes one command and returns.
  // The main loop() will call it repeatedly.

  // Don't do anything if there are no commands or if paused.
  if (!executing || paused || commandCount == 0) {
    return;
  }

  // If audio from a previous command is playing, wait for it to finish.
  if (isPlayingAudio) {
    return;
  }

  // Check if we have executed all commands in the buffer
  if (currentCommandIndex >= commandCount) {
    Serial.println("✅ Execution finished. Playing completion sound...");
    playAudio("/sdcard/complete.wav"); 
    
    // Reset for next execution
    executing = false;
    currentCommandIndex = 0;
    // Optional: clear commands after execution, or keep them to run again
    // commandCount = 0; 
    return;
  }

  // Execute the next command in the buffer
  Serial.printf("Executing command %d of %d...\n", currentCommandIndex + 1, commandCount);
  executeSingleCommand(commandBuffer[currentCommandIndex]);
  currentCommandIndex++;
}

void executeSingleCommand(int command) {
    if (!gyro) return;
    switch (command) {
        case CMD_FORWARD:
            Serial.println("Executing: Forward");
            moveForward(); 
            vTaskDelay(pdMS_TO_TICKS(MOVETIME)); 
            stopMotors(); 
            break;
        case CMD_BACKWARD:
            Serial.println("Executing: Backward");
            moveBackward(); 
            vTaskDelay(pdMS_TO_TICKS(MOVETIME)); 
            stopMotors(); 
            break;
        case CMD_TURN_90_LEFT:
            Serial.println("Executing: Turn 90 Left");
            if (!gyro->turnToAngle(-90, turnLeft, turnRight, stopMotors)) {
                turnLeft(); 
                vTaskDelay(pdMS_TO_TICKS(TURNTIME));
            }
            stopMotors();
            break;
        case CMD_TURN_90_RIGHT:
            Serial.println("Executing: Turn 90 Right");
            if (!gyro->turnToAngle(90, turnLeft, turnRight, stopMotors)) {
                turnRight(); 
                vTaskDelay(pdMS_TO_TICKS(TURNTIME));
            }
            stopMotors();
            break;
        case CMD_PLAY_RECORDING:
            if (hasRecording) {
                const char* recording_path = "/sdcard/user_recording.wav";
                // Check if the recording file is valid before trying to play it
                FILE* f = fopen(recording_path, "rb");
                if (f) {
                    fseek(f, 0, SEEK_END);
                    long fileSize = ftell(f);
                    fclose(f);
                    if (fileSize > 44) { // Basic check: bigger than a WAV header
                        Serial.println("Executing: Play Recording");
                        playAudio(recording_path);
                        // The executeCommands loop will wait for this to finish
                    } else {
                        Serial.printf("Skipping: Recording file is invalid or empty (size: %ld bytes)\n", fileSize);
                        hasRecording = false; // Mark as no valid recording
                    }
                } else {
                    Serial.println("Skipping: Could not open recording file");
                    hasRecording = false;
                }
            } else {
                Serial.println("Skipping: No recording available");
            }
            break;
    }
}

void performHeadVibration() {
  if (isVibrating) return;
  isVibrating = true;
  unsigned long startTime = millis();
  bool direction = true;
  while (millis() - startTime < VIBRATE_TIME) {
    if (direction) moveForward();
    else moveBackward();
    vTaskDelay(pdMS_TO_TICKS(VIBRATE_INTERVAL));
    direction = !direction;
  }
  stopMotors();
  isVibrating = false;
}

void moveForward() {
  digitalWrite(MOTOR_IN1, HIGH); digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, HIGH); digitalWrite(MOTOR_IN4, LOW);
}

void moveBackward() {
  digitalWrite(MOTOR_IN1, LOW); digitalWrite(MOTOR_IN2, HIGH);
  digitalWrite(MOTOR_IN3, LOW); digitalWrite(MOTOR_IN4, HIGH);
}

void turnLeft() {
  digitalWrite(MOTOR_IN1, LOW); digitalWrite(MOTOR_IN2, HIGH);
  digitalWrite(MOTOR_IN3, HIGH); digitalWrite(MOTOR_IN4, LOW);
}

void turnRight() {
  digitalWrite(MOTOR_IN1, HIGH); digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, LOW); digitalWrite(MOTOR_IN4, HIGH);
}

void stopMotors() {
  digitalWrite(MOTOR_IN1, LOW); digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, LOW); digitalWrite(MOTOR_IN4, LOW);
}

// ====================================================================================
//                          MULTITOUCH AND EMOTION FUNCTIONS
// ====================================================================================

// Count the number of bits set in a value (number of buttons pressed)
uint8_t countBitsSet(uint16_t value) {
  uint8_t count = 0;
  while (value) {
    count += value & 1;
    value >>= 1;
  }
  return count;
}

// Handle multitouch combinations
void handleMultitouch(uint16_t touchMask) {
  Serial.printf("🎭 EXECUTING MULTITOUCH: mask=0x%04X\n", touchMask);
  
  // Check if this is an emotion combination
  if (checkEmotionCombination(touchMask)) {
    return; // Emotion was played, don't process as regular multitouch
  }
  
  // Handle other multitouch combinations
  switch (touchMask) {
    case 0x03: // Buttons 1+2 (Forward+Backward)
      Serial.println("🎭 Multitouch: Emergency Stop");
      playAudio("/sdcard/emergency_stop.wav");
      executing = false;
      paused = false;
      stopMotors();
      break;
      
    case 0x0C: // Buttons 3+4 (Left+Right) 
      Serial.println("🎭 Multitouch: Calibrate Gyroscope");
      playAudio("/sdcard/calibrating.wav");
      if (gyro) {
        gyro->initialize(); // Re-calibrate
      }
      break;
      
    case 0x30: // Buttons 5+6 (Delete+Record)
      Serial.println("🎭 Multitouch: Clear All + Factory Reset");
      playAudio("/sdcard/factory_reset.wav");
      commandCount = 0;
      hasRecording = false;
      break;
      
    case 0xC0: // Buttons 7+8 (Start+Head)
      Serial.println("🎭 Multitouch: System Info");
      playAudio("/sdcard/system_info.wav");
      Serial.printf("Commands: %d, Recording: %s, Executing: %s\n", 
                   commandCount, hasRecording ? "Yes" : "No", executing ? "Yes" : "No");
      break;
      
    default:
      Serial.printf("🎭 Unknown multitouch combination: 0x%04X\n", touchMask);
      playAudio("/sdcard/beep.wav"); // Use beep instead of unknown.wav
      break;
  }
}

// Check if the touch combination triggers an emotion
bool checkEmotionCombination(uint16_t touchMask) {
  // New emotion combinations as requested: 1+8, 1+5, 1+6+8, 1+5+6+7+8
  switch (touchMask) {
    case 0x81: // Buttons 1+8 (Forward+Head)
      playEmotion("happy");
      return true;
      
    case 0x11: // Buttons 1+5 (Forward+Delete)
      playEmotion("sad");
      return true;
      
    case 0xA1: // Buttons 1+6+8 (Forward+Record+Head) - 0x01 + 0x20 + 0x80 = 0xA1
      playEmotion("excited");
      return true;
      
    case 0xF1: // Buttons 1+5+6+7+8 (Forward+Delete+Record+Start+Head) - 0x01 + 0x10 + 0x20 + 0x40 + 0x80 = 0xF1
      playEmotion("love");
      return true;
      
    default:
      return false; // Not an emotion combination
  }
}

// Play an emotion sequence (AUDIO ONLY - no robot movements)
void playEmotion(const char* emotionName) {
  if (emotionPlaying) {
    Serial.println("🎭 Emotion already playing, ignoring new emotion");
    return;
  }
  
  emotionPlaying = true;
  Serial.printf("🎭 PLAYING EMOTION: %s\n", emotionName);
  
  // Map emotion names to existing audio files on SD card
  const char* audioFile = "/sdcard/beep.wav"; // Default fallback
  
  if (strcmp(emotionName, "happy") == 0) {
    audioFile = "/sdcard/greeting.wav"; // Use greeting for happy
  } else if (strcmp(emotionName, "sad") == 0) {
    audioFile = "/sdcard/stop.wav"; // Use stop for sad
  } else if (strcmp(emotionName, "excited") == 0) {
    audioFile = "/sdcard/start.wav"; // Use start for excited
  } else if (strcmp(emotionName, "love") == 0) {
    audioFile = "/sdcard/complete.wav"; // Use complete for love
  }
  
  // Play the emotion audio
  playAudio(audioFile);
  
  // Create simple task to reset emotion flag when audio finishes
  xTaskCreate([](void* param) {
    // Wait for audio to finish
    while (isPlayingAudio) {
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    emotionPlaying = false;
    Serial.printf("🎭 EMOTION COMPLETE: %s\n", (const char*)param);
    vTaskDelete(NULL);
  }, "EmotionTask", 2048, (void*)emotionName, 3, NULL);
}
