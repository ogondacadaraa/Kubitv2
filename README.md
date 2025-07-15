# ESP32 Kubit Robot Project

A sophisticated ESP32-based robot with audio capabilities, gyroscope control, emotion system, and touch interface. This project has been optimized to eliminate all runtime deprecation warnings while maintaining full functionality.

## 🎯 Features

### Core Functionality
- **Touch Interface**: TTP229 16-key touch keypad for robot control
- **Motor Control**: L298N motor driver for precise movement
- **Gyroscope Control**: MPU6050 for accurate turning and orientation
- **Audio System**: I2S audio output for sound effects and feedback
- **Emotion System**: Interactive emotion responses based on button patterns
- **Command Buffer**: Program and execute sequences of movements

### Audio Capabilities
- **Sound Effects**: Audio feedback for all robot actions
- **File Support**: WAV file playback from SD card
- **Volume Control**: Adjustable audio levels
- **I2S Output**: High-quality digital audio output

### Storage
- **SD Card Support**: Store audio files and data
- **SPIFFS Alternative**: Internal flash storage option

## 🚀 Hardware Requirements

### Main Components
- **ESP32 Development Board** (ESP32-WROOM-32)
- **TTP229 Touch Keypad** (16-key capacitive touch)
- **MPU6050 Gyroscope/Accelerometer**
- **L298N Motor Driver**
- **DC Motors** (2x for differential drive)
- **I2S Audio Amplifier** (e.g., MAX98357A)
- **Speaker** (4-8 ohm)
- **MicroSD Card Module**

### Pin Configuration

#### Touch Keypad (TTP229)
- SCL: GPIO 33
- SDA: GPIO 32

#### Motors (L298N)
- IN1: GPIO 23
- IN2: GPIO 2
- IN3: GPIO 12
- IN4: GPIO 13

#### I2S Audio Output
- BCLK: GPIO 19
- LRCLK: GPIO 27
- DIN: GPIO 18

#### SD Card (SPI)
- MISO: GPIO 4
- MOSI: GPIO 15
- CLK: GPIO 14
- CS: GPIO 25

#### Gyroscope (I2C)
- SDA: GPIO 21
- SCL: GPIO 22

## 🛠️ Software Setup

### Prerequisites
- [PlatformIO](https://platformio.org/) installed
- ESP32 development framework
- Git for version control

### Build and Upload
```bash
# Clone the repository
git clone <your-repo-url>
cd 250622-132811-esp32dev

# Build the project
platformio run

# Upload to ESP32
platformio run --target upload

# Monitor serial output
platformio device monitor --baud 115200
```

## 🎮 Controls

### Button Mapping
1. **Button 1**: Forward movement
2. **Button 2**: Backward movement  
3. **Button 3**: Turn 90° left
4. **Button 4**: Turn 90° right
5. **Button 5**: Delete all commands
6. **Button 6**: Record voice (disabled)
7. **Button 7**: Start/Stop/Pause execution
8. **Button 8**: Head vibration

### Operation Modes
- **Programming Mode**: Add movement commands to buffer
- **Execution Mode**: Run programmed sequence
- **Emotion Mode**: Triggered by specific button patterns

## 🧠 Emotion System

The robot recognizes button patterns and responds with emotions:
- **CONFUSED**: Rapid button presses (3+ times)
- **HAPPY**: Specific sequences
- **EXCITED**: Pattern recognition
- Each emotion triggers unique movements and sounds

## 📁 Project Structure

```
250622-132811-esp32dev/
├── src/
│   ├── main.cpp              # Main application code
│   ├── config.h              # Pin definitions and settings
│   └── config.cpp            # Configuration implementation
├── lib/
│   ├── ADCSampler/           # (Unused - removed for deprecation fix)
│   ├── DACOutput/            # (Unused - removed for deprecation fix)
│   ├── EmotionSystem/        # Emotion recognition and responses
│   ├── GyroscopeControl/     # MPU6050 gyroscope control
│   ├── I2SOutput/            # I2S audio output
│   ├── InfraredSensors/      # (Unused - removed for deprecation fix)
│   ├── LineFollower/         # (Unused - removed for deprecation fix)
│   ├── Output/               # Audio output base class
│   ├── SDCard/               # SD card file system
│   ├── SPIFFS/               # Internal flash storage
│   ├── WAVFileReader/        # WAV audio file reader
│   └── WAVFileWriter/        # WAV audio file writer
├── include/
├── test/
├── platformio.ini            # PlatformIO configuration
└── README.md                 # This file
```

## ✅ Optimization Notes

This project has been specifically optimized to:
- **Eliminate Runtime Warnings**: No legacy ADC or I2S driver warnings
- **Clean Boot Sequence**: Minimal startup messages
- **Efficient Memory Usage**: Optimized for ESP32 constraints
- **Modular Design**: Easy to extend and modify

### Removed Components
To eliminate deprecation warnings, the following components were removed:
- Infrared sensors and line following
- ADC-based microphone input
- Legacy I2S microphone sampling
- Unused audio input components

## 🔧 Troubleshooting

### Common Issues
1. **No Audio Output**: Check I2S wiring and audio files on SD card
2. **Touch Not Working**: Verify TTP229 connections and power
3. **Motors Not Moving**: Check L298N wiring and power supply
4. **Gyroscope Issues**: Ensure I2C connections and calibration

### Serial Monitor
Monitor at 115200 baud for debug information:
- Boot sequence status
- Component initialization
- Button press detection
- Command execution
- Error messages

## 📜 License

This project is open source. Feel free to modify and distribute.

## 🤝 Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues for bugs and feature requests.

## 📞 Support

For questions or issues, please check the documentation or open an issue in the repository.
