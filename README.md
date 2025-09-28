## 🕷️ Hexapod Robot Project

6-legged hexapod robot with tripod gait locomotion on ESP32-S3-DevKitC-1

## Quick Start

1. **Hardware**: ESP32-S3 + 32-channel servo controller + 18x MG90 servos
2. **Upload**: Flash `hexapod.ino` to ESP32
3. **Connect**: Join WiFi "Homenet_plus", open IP address in browser
4. **Test**: Use web interface for diagnostics and movement

## Key Files

- `hexapod.ino` - Main program
- `config.h` - Hardware mapping & calibration  
- `commands.h` - Servo control functions
- `page_html.h` - Web control interface
- `safety.h/.cpp` - Safety systems
- `promt` - Complete project documentation for AI assistance

## Testing Sequence  

1. **DIAGNOSTIC** - Test all 32 servos
2. **JOINT_TEST** - Individual leg movement testing
3. **TRIPOD_TEST** - Group lifting behavior
4. **FWD** - Forward movement (in development)

## Current Status

✅ **Working**: WiFi, servo control, individual testing, tripod group lifting  
🔧 **In Progress**: Full walking locomotion with forward progression

For detailed debugging information, see `promt` file.