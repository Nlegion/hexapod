# 🔧 Hexapod Debug & Troubleshooting Guide

## 🚀 **System Status: Production Ready**

This guide covers troubleshooting for the fully validated hexapod robot system. Most issues should be rare due to comprehensive testing and error handling.

## 🧪 **Quick Diagnostic Sequence**

### 1. **Pre-Upload Validation**
```bash
# Always run before uploading to ESP32
./run_tests.bat      # Windows
./run_tests.sh       # Linux/Mac

# Expected: "Results: 24 PASSED, 0 FAILED"
```

### 2. **Hardware Upload & Initialization**
1. Upload `hexapod.ino` to ESP32-S3
2. Monitor Serial output at 115200 baud
3. Expected sequence:
   ```
   [INFO] Servo controller initialized successfully  
   [INFO] Connected. IP: 192.168.1.xxx
   [INFO] Setup complete. Servos will be reset in main loop
   [INFO] Ready. All servos in neutral position
   ```

### 3. **Web Interface Testing**
1. Connect to robot's IP address
2. Test basic commands in order:
   - **RESET** → All legs to neutral position
   - **DIAGNOSTIC** → Test all 32 servo channels
   - **JOINT_TEST** → Verify individual leg movements
   - **TRIPOD_TEST** → Check group coordination with live coordinates

## ⚠️ **Common Issues & Solutions**

### 🔌 **Connection Problems**

**WiFi Connection Fails:**
- System automatically falls back to AP mode "Hexapod_Config" (password: "12345678")
- Connect device to this network and access 192.168.4.1
- Check Serial output for connection details

**Web Interface Not Loading:**  
- Verify IP address from Serial monitor
- Try AP mode fallback (automatic after 15 seconds)
- Ensure device is on same network as ESP32

### ⚙️ **Servo Control Issues**

**Servo Controller Not Responding:**
```
[ERROR] Failed to initialize servo controller. Result: X
[ERROR] System cannot continue without servo controller
```
- **Solution**: Check power supply, wiring (Serial1 pins 4,5), servo controller power
- **Verify**: Servo controller receives 9600 baud serial communication

**Individual Servos Not Moving:**
- Run **DIAGNOSTIC** command to test all 32 channels
- Check specific servo power, wiring, channel mapping in `config.h`
- Verify pulse values are within 1000-2000μs range

**Incorrect Movement Directions:**
- **Left legs only**: System correctly inverts COXA only (`LEG_LIFT_DIRECTIONS`)
- **All legs wrong**: Check hardware wiring, servo orientation
- **Random legs**: Verify `LEG_SERVO_MAP` channel assignments in `config.h`

## 📊 **Diagnostic Commands Reference**

| Command | Purpose | Expected Result |
|---------|---------|-----------------|
| `RESET` | Initialize all servos | All legs to neutral position |
| `DIAGNOSTIC` | Test all 32 channels | Sequential servo movement 1-32 |
| `JOINT_TEST` | Individual leg testing | Each leg joint moves correctly |
| `TRIPOD_TEST` | High-lift group coordination | Live coordinates show enhanced lifting height |
| `FWD` | Forward locomotion | High-clearance gait with visible progression |
|| `FAST` | Speed mode | 120ms cycle time, rapid movement |
|| `NORMAL` | Standard mode | 150ms cycle time, balanced performance |
|| `SLOW` | Careful mode | 200ms cycle time, precise movement |
| `EMERGENCY` | Emergency stop | Immediate halt of all movement |

---

## 📞 **Additional Resources**

- **Complete Documentation**: `promt` file contains full technical details
- **Testing Guide**: `TESTING.md` for unit test framework usage  
- **Source Code**: All files extensively commented

### 🎯 **Remember: System is Production Ready**

Most debugging should focus on mechanical calibration and performance optimization rather than fundamental system issues. All core functionality has been thoroughly tested and validated.