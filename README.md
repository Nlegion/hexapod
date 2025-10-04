# 🕷️ Hexapod Robot - 6-Legged Walking Robot

## 📋 Project Overview

Production-ready hexapod robot with tripod gait locomotion, inverse kinematics, and web-based control interface. Built on ESP32-S3 with 32-channel servo controller.

**Current Version:** 4.0 (Clean Architecture + FreeRTOS)  
**Status:** ✅ Production Ready  
**Architecture:** Clean Architecture + Multithreading  
**Tests:** ✅ **38/38 PASSED** (включая Clean Architecture validation)

---

## ✨ What's New in v4.0

### 🏗️ Clean Architecture Implementation
Проект полностью мигрирован на Clean Architecture с разделением на слои:
- **Core Layer** - типы, конфигурация, логирование
- **Domain Layer** - бизнес-логика (entities, services, repositories interfaces)
- **Application Layer** - Use Cases и координация
- **Infrastructure Layer** - реализации для hardware
- **DI Container** - управление зависимостями

### ⚡ FreeRTOS Multithreading
Асинхронная архитектура с 3 tasks на 2 ядрах ESP32:
- Gait Task (Core 0) - обновление походки 20Hz
- Web Task (Core 1) - WebSocket + HTTP обработка
- Battery Task (Core 0) - мониторинг батареи

### 📊 Architecture Comparison

| Aspect | Old (hexapod_legacy.ino) | **New (hexapod.ino)** |
|--------|--------------------------|------------------------|
| Lines | 963 lines | **~300 lines main** |
| Architecture | Monolithic | **Layered (Clean)** |
| Testability | Сложно | **Легко (DI + Mocks)** |
| Threading | Blocking loop | **FreeRTOS tasks** |
| Dependencies | Tight coupling | **Loose coupling (DI)** |
| Файлов | 8 файлов | **20+ файлов (организовано)** |

---

## 🚀 Quick Start

### 1. Hardware Requirements
- ESP32-S3-DevKitC-1 microcontroller
- 32-channel PWM servo controller (RTRobot compatible)
- 18× MG90 micro servos (3 per leg)
- 3S LiPo battery (11.1V, recommended capacity: 2200mAh+)
- Hexapod chassis with 6 legs

### 2. Software Setup
```bash
# Clone the repository
git clone <repo-url>
cd hexapod

# Upload to ESP32
Arduino IDE → Open hexapod.ino → Upload

# Or use legacy version
Arduino IDE → Open hexapod_legacy.ino → Upload
```

### 3. Run Tests (Optional but Recommended)
```bash
# Windows
.\run_tests.bat

# Linux/Mac
chmod +x run_tests.sh
./run_tests.sh
```

Expected: **38/38 PASSED** ✅
- 29 legacy integration tests
- 9 Clean Architecture validation tests

### 4. First Run
1. Connect to WiFi network or AP mode (`Hexapod_Config`)
2. Open web interface at robot's IP address
3. Execute: `RESET` → `FWD` → Test movement

---

## 📁 Project Structure

### 🆕 NEW: Clean Architecture Edition

```
hexapod/
├── hexapod.ino              # 🆕 Main program (Clean Architecture + FreeRTOS)
├── hexapod_legacy.ino       # Legacy version (monolithic, still works)
│
├── src/                     # 🏗️ Clean Architecture Source
│   ├── core/                      # ⚙️ Core Layer
│   │   ├── Types.h                    # Common types & enums
│   │   ├── Config.h                   # Hardware configuration
│   │   └── Logger.h                   # Logging system
│   │
│   ├── domain/                    # 🎯 Domain Layer (Business Logic)
│   │   ├── entities/                  # Domain entities
│   │   ├── services/                  # Domain services (IK, Gait, Safety)
│   │   └── repositories/              # Repository interfaces
│   │
│   ├── application/               # 📋 Application Layer (Use Cases)
│   │   ├── usecases/                  # Use case implementations
│   │   ├── dto/                       # Data Transfer Objects
│   │   └── RobotController.h          # Main controller
│   │
│   ├── infrastructure/            # 🔧 Infrastructure (Hardware)
│   │   └── hardware/                  # Hardware implementations
│   │
│   └── di/                        # 💉 Dependency Injection
│       └── Container.h                # DI Container
│
├── [Legacy files]           # Old architecture files (config.h, etc.)
│
├── docs/                    # 📚 Documentation
│   ├── COMMAND_REFERENCE.md      # Complete command list
│   ├── QUICK_REFERENCE.md        # Quick reference card
│   ├── INDEX.md                  # Documentation navigation
│   │
│   ├── features/                 # Feature implementations
│   │   ├── VERSION_3.0_SUMMARY.md
│   │   ├── TRAJECTORY_FIX_REPORT.md
│   │   ├── LEGACY_FEATURES_IMPLEMENTATION.md
│   │   ├── FIX_REPORT.md
│   │   ├── IMPLEMENTATION_SUMMARY.md
│   │   └── BATTERY_INDICATOR_REPORT.md
│   │
│   ├── hardware/                 # Hardware documentation
│   │   ├── BATTERY_MONITORING_SETUP.md
│   │   ├── SIMULATION_MODE_GUIDE.md
│   │   ├── DEBUG_GUIDE.md
│   │   └── RTRobot-Servo-Motor-Controller-User-Instruction.pdf
│   │
│   ├── troubleshooting/          # Problem solving
│   │   ├── BATTERY_TROUBLESHOOTING.md
│   │   ├── QUICK_FIX_0.03V.md
│   │   ├── DIAGNOSTIC_RESULT_ANALYSIS.md
│   │   └── FIX_STEPS.md
│   │
│   └── testing/                  # Testing system
│       └── TEST_SYSTEM_README.md
│
├── tests/                   # 🧪 Unit testing framework
│   ├── run_tests.cpp             # Test runner
│   ├── test_hexapod.cpp          # Hexapod tests
│   ├── test_*.h                  # Test modules
│   ├── run_tests.bat/.sh         # Test scripts
│   └── Makefile
│
├── legacy_source/           # 🗄️ Original project (reference only)
│   ├── spider.ino
│   └── page_html.h
│
├── run_tests.bat/.sh        # Quick test execution
└── promt                    # AI assistant prompt
```

### 🎯 Clean Architecture Benefits

The new **Clean Architecture + FreeRTOS** implementation provides:

**Architecture:**
- ✅ **Separation of Concerns** - каждый слой отвечает за свою область
- ✅ **Dependency Inversion** - зависимости направлены к центру (Domain)
- ✅ **Testability** - легко тестировать с mock объектами
- ✅ **Maintainability** - файлы < 300 строк, легко понять
- ✅ **Scalability** - легко добавлять новые Use Cases

**Multithreading (FreeRTOS):**
- ⚡ **Task 1 (Core 0, Priority 2)**: Gait Control - обновление походки (20Hz)
- ⚡ **Task 2 (Core 1, Priority 1)**: Web Server - WebSocket + HTTP
- ⚡ **Task 3 (Core 0, Priority 0)**: Battery Monitor - мониторинг батареи

**Performance:**
- 🚀 Истинная многозадачность на 2 ядрах ESP32
- 🚀 Неблокирующая архитектура
- 🚀 Priority-based scheduling

---

## 🎮 Control Commands

### Basic Movement
```
FWD     - Move forward (tripod gait)
BWD     - Move backward  
LEFT    - Turn left (rotate in place)
RIGHT   - Turn right (rotate in place)
STOP    - Stop movement & return to neutral
```

### Speed Control
```
FAST    - Fast gait (120ms/step)
NORMAL  - Normal speed (150ms/step) [default]
SLOW    - Slow gait (200ms/step)
```

### Gestures & Tricks
```
SHAKE   - Shake hand (right front leg)
WAVE    - Wave (left front leg)
```

### Body Adjustments
```
BODY_UP/DOWN    - Raise/lower body height
HEAD_UP/DOWN    - Tilt head up/down
LEAN_LEFT/RIGHT - Lean body sideways
TWIST_LEFT/RIGHT - Twist body rotation
```

### Diagnostics
```
RESET       - Return to startup position
CALIBRATE   - Calibration mode
DIAGNOSTIC  - Test all 32 servo channels
JOINT_TEST  - Test joint directions
TRIPOD_TEST - Test tripod gait
TEST_LEG_0..5 - Test individual legs
BATTERY_CHECK - Battery diagnostics
EMERGENCY   - Emergency stop
```

**📖 Full command reference:** [`docs/COMMAND_REFERENCE.md`](docs/COMMAND_REFERENCE.md)

---

## 🔧 Key Features

### Version 3.0 Highlights
- ✅ **+80% Movement Speed** - Increased COXA amplitude (±100 → ±180)
- ✅ **Perfect Trajectory Symmetry** - Straight forward movement
- ✅ **Enhanced Gestures** - SHAKE (+87%), WAVE (+175%) more expressive
- ✅ **29 Unit Tests** - Complete test coverage
- ✅ **Battery Monitoring** - Real-time voltage display (with simulation mode)

### Core Systems
- **Inverse Kinematics (IK)** - Precise leg positioning
- **Forward Kinematics (FK)** - Position calculation
- **Tripod Gait** - Stable 3-leg locomotion pattern
- **Non-blocking Architecture** - Responsive control
- **Web Interface** - WiFi/AP mode control panel
- **Safety Systems** - Multi-layer protection
- **Comprehensive Logging** - Debug/Info/Warning/Error levels

---

## 🧪 Testing

### Run Tests Locally
```bash
# Windows
run_tests.bat

# Linux/Mac
./run_tests.sh
```

**Results:** 29/29 tests passing ✅

### Test Categories
- Configuration validation
- Inverse/Forward kinematics
- Trajectory verification
- Safety systems
- Servo communication
- Gait cycle simulation

**📖 Testing guide:** [`docs/testing/TEST_SYSTEM_README.md`](docs/testing/TEST_SYSTEM_README.md)

---

## 📊 Performance Metrics

| Metric | Version 2.0 | Version 3.0 | Improvement |
|--------|-------------|-------------|-------------|
| **COXA Amplitude** | ±100 | ±180 | +80% 🚀 |
| **Movement Speed** | Base | +80% | 1.8× faster |
| **SHAKE Expressiveness** | Base | +87% | More visible |
| **WAVE Expressiveness** | Base | +175% | Much better |
| **Trajectory Symmetry** | Partial | Perfect | 100% |
| **Tests** | 25 | 29 | +16% |

---

## 🔌 Hardware Specifications

### Microcontroller
- **MCU:** ESP32-S3-DevKitC-1
- **Communication:** Serial1 (pins 4,5) → Servo controller @ 9600 baud
- **Network:** WiFi (STA/AP mode)

### Servo System
- **Controller:** 32-channel PWM (RTRobot compatible)
- **Servos:** 18× MG90 micro servos
- **Pulse Range:** 1000-2000μs (500μs swing = 180°)
- **Protocol:** `#<channel>P<pulse>T<time>\r\n`

### Power
- **Battery:** 3S LiPo (11.1V nominal, 12.6V max)
- **Monitoring:** ADC-based with voltage divider (4.2:1)
- **Critical Voltage:** < 9.5V (auto-warning)

**📖 Hardware details:** [`docs/hardware/`](docs/hardware/)

---

## 🛠️ Configuration & Calibration

### Key Configuration Files

**`config.h`** - Main configuration:
- Servo channel mapping (`LEG_SERVO_MAP`)
- Calibration offsets (`LEG_OFFSETS`)
- Movement directions (`LEG_FORWARD_DIRECTIONS`, `LEG_LIFT_DIRECTIONS`)
- Gait trajectories (`TRANSFER_TRAJ`, `SUPPORT_TRAJ`)
- Physical dimensions (leg lengths, body radius)
- Safety limits (pulse ranges, angle constraints)

### Calibration Process
1. Upload code with default configuration
2. Run `RESET` command
3. Use `TEST_LEG_0` through `TEST_LEG_5` to test individual legs
4. Adjust `LEG_OFFSETS` in `config.h` as needed
5. Re-upload and verify with `JOINT_TEST`

**📖 Calibration guide:** [`docs/troubleshooting/FIX_STEPS.md`](docs/troubleshooting/FIX_STEPS.md)

---

## 🌐 Web Interface

### Features
- Real-time battery monitoring
- Movement controls (forward/backward/turns)
- Speed adjustment (Fast/Normal/Slow)
- Gesture buttons (Shake/Wave)
- Body adjustment controls
- Diagnostic tools
- Individual leg testing
- Live coordinate display (during TRIPOD_TEST)

### Access
- **WiFi Mode:** Connect to network, access via robot IP
- **AP Mode:** Connect to "Hexapod_Config" (password: 12345678), access via `192.168.4.1`

---

## 📚 Documentation Index

### Getting Started
- [`README.md`](README.md) - This file (project overview)
- [`docs/QUICK_REFERENCE.md`](docs/QUICK_REFERENCE.md) - Quick command reference
- [`docs/INDEX.md`](docs/INDEX.md) - Complete documentation index

### Feature Documentation
- [`docs/features/VERSION_3.0_SUMMARY.md`](docs/features/VERSION_3.0_SUMMARY.md) - Latest version summary
- [`docs/features/TRAJECTORY_FIX_REPORT.md`](docs/features/TRAJECTORY_FIX_REPORT.md) - Trajectory optimization
- [`docs/features/LEGACY_FEATURES_IMPLEMENTATION.md`](docs/features/LEGACY_FEATURES_IMPLEMENTATION.md) - Legacy features port

### Hardware & Setup
- [`docs/hardware/BATTERY_MONITORING_SETUP.md`](docs/hardware/BATTERY_MONITORING_SETUP.md) - Battery setup
- [`docs/hardware/SIMULATION_MODE_GUIDE.md`](docs/hardware/SIMULATION_MODE_GUIDE.md) - Battery simulation
- [`docs/hardware/DEBUG_GUIDE.md`](docs/hardware/DEBUG_GUIDE.md) - Debug procedures

### Troubleshooting
- [`docs/troubleshooting/BATTERY_TROUBLESHOOTING.md`](docs/troubleshooting/BATTERY_TROUBLESHOOTING.md) - Battery issues
- [`docs/troubleshooting/QUICK_FIX_0.03V.md`](docs/troubleshooting/QUICK_FIX_0.03V.md) - Common battery fix

### Testing
- [`docs/testing/TEST_SYSTEM_README.md`](docs/testing/TEST_SYSTEM_README.md) - Testing framework

---

## 🤝 Contributing

### Development Workflow
1. Make changes to code
2. Run tests: `run_tests.bat` (Windows) or `./run_tests.sh` (Linux/Mac)
3. Ensure all 29 tests pass
4. Test on hardware if possible
5. Update documentation

### Code Style
- Use descriptive variable names
- Add comments for complex logic
- Follow existing code structure
- Update tests for new features

---

## 📝 License

[Specify your license here]

---

## 🔗 Links

- **Documentation:** [`docs/INDEX.md`](docs/INDEX.md)
- **Command Reference:** [`docs/COMMAND_REFERENCE.md`](docs/COMMAND_REFERENCE.md)
- **Latest Release:** [`docs/features/VERSION_3.0_SUMMARY.md`](docs/features/VERSION_3.0_SUMMARY.md)

---

## 📞 Support

For issues, questions, or contributions:
- Check [`docs/troubleshooting/`](docs/troubleshooting/) for common problems
- Review [`docs/INDEX.md`](docs/INDEX.md) for complete documentation
- Check serial monitor (115200 baud) for debug logs

---

## 🎯 Current Status

**Version:** 3.0  
**Status:** ✅ Production Ready  
**Tests:** 29/29 Passing  
**Last Updated:** 2025-10-03

**Ready for deployment!** 🚀

---

*Made with ❤️ for robotics enthusiasts*
