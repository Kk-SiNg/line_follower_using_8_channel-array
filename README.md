# Line Maze Solver - 8-Channel RLS-08 Edition

🏆 **Optimized for IIT Bombay Mesmerize Competition**

## Overview

Advanced line maze solver using ESP32 and 8-channel RLS-08 hybrid sensor array (6 analog + 2 digital). Features adaptive speed control, junction prediction, and competition-grade performance.

## Hardware Requirements

### Main Components
- **ESP32 38-pin DevKit** (any variant with ADC1 pins)
- **RLS-08 8-Channel Sensor Array** (SmartElex or similar)
- **L298N Motor Driver**
- **DC Motors with Encoders** (2x)
- **Power Supply** (7.4V 2S LiPo recommended)
- **Chassis** with proper sensor mounting

### Pin Configuration

#### Sensor Array (8 channels)
| Sensor | GPIO | Type | Function |
|--------|------|------|----------|
| S1 | 25 | Digital | Rightmost (junction detection) |
| S2 | 39 | Analog | Line following |
| S3 | 34 | Analog | Line following |
| S4 | 35 | Analog | Center-right (critical) |
| S5 | 32 | Analog | Center-left (critical) |
| S6 | 33 | Analog | Line following |
| S7 | 36 | Analog | Line following |
| S8 | 26 | Digital | Leftmost (junction detection) |

#### Motors & Encoders
- Left Motor: IN1=15, IN2=4, PWM=16
- Right Motor: IN1=17, IN2=18, PWM=19
- Left Encoder: A=21, B=22
- Right Encoder: A=23, B=5

## Software Setup

### 1. Install Dependencies
```bash
# Arduino IDE
- ESP32 Board Package (2.0.0+)
- No external libraries required (uses native WiFi)

# PlatformIO
platformio.ini already configured
```

### 2. Configure WiFi
Edit `src/Pins.h`:
```cpp
#define SSID "YourNetworkName"
#define PASSWORD "YourPassword"
```

### 3. Upload Code
```bash
# Arduino IDE: Select ESP32 Dev Module, Upload
# PlatformIO: pio run -t upload
```

## Testing Procedure

### Phase 1: Hardware Verification (15 minutes)

#### Test 1: Basic Sensor Reading
```bash
Upload: tests/Test_RLS08_Basic/Test_RLS08_Basic.ino
```
**Expected:**
- All 8 sensors respond
- WHITE surface → HIGH (4095 for analog, ON for digital)
- BLACK surface → LOW (0 for analog, OFF for digital)

**Troubleshooting:**
- Sensor stuck HIGH: Turn potentiometer CCW (less sensitive)
- Sensor stuck LOW: Check power connections, adjust height

#### Test 2: Calibration
```bash
Upload: tests/Test_RLS08_Calibration/Test_RLS08_Calibration.ino
```
**Expected:**
- Range > 1500 for "Excellent" quality
- Range > 1000 minimum acceptable

**Actions:**
- Adjust potentiometers for optimal contrast
- Test at different heights (3-5mm optimal)

#### Test 3: Position Calculation
```bash
Upload: tests/Test_RLS08_Position/Test_RLS08_Position.ino
```
**Expected:**
- Position smoothly changes from -7.0 (right) to +7.0 (left)
- No jumps or dead zones
- Decimal precision visible

#### Test 4: Junction Detection
```bash
Upload: tests/Test_RLS08_Junction/Test_RLS08_Junction.ino
```
**Expected:**
- Straight line: NO junction detected
- T-junction: Correct paths detected
- No false positives during normal line following

**Critical Test:**
Place sensor on 3cm straight line and verify:
```
Active sensors: 3-4 (center sensors only)
Junction: NO
Type: Straight Line
```

### Phase 2: Integration Testing (30 minutes)

#### Test 5: Complete System
```bash
Upload: tests/Test_RLS08_Hybrid/Test_RLS08_Hybrid.ino
```

Verify:
- [ ] Auto-calibration completes
- [ ] Position calculation smooth
- [ ] PID output reasonable
- [ ] Simulated motor speeds correct

### Phase 3: Main Code Testing (Practice Maze)

1. **Upload main code** (`src/main.cpp`)
2. **Calibration**: Robot rotates, place on line during calibration
3. **Profile**: Confirm `PROFILE_TEST_MAZE` active
4. **Run 1 (Mapping)**:
   - Press button or send "START" via WiFi
   - Robot follows line using LSRB algorithm
   - Verify junction detection at each turn
   - Robot stops at line end automatically
5. **Optimization**: Path automatically optimized
6. **Run 2 (Solving)**:
   - Press button again
   - Robot follows optimized path at high speed
   - Completes maze faster

## Competition Features

### 1. Adaptive Speed Control
Automatically adjusts speed based on line curvature:
- Straight: HIGH_SPEED (255)
- Slight curve: BASE_SPEED (150)
- Sharp turn: Reduced speed

### 2. Junction Prediction
Predicts upcoming junctions based on average segment length, slows down for accuracy.

### 3. Sensor Confidence
Automatically weights sensors by calibration quality, ignores unreliable sensors.

### 4. Dynamic Debounce
Adjusts junction detection timing based on current speed.

### 5. Line Recovery
Attempts automatic recovery if line is temporarily lost.

### 6. Battery Compensation
*(Optional)* Compensates motor speeds for voltage drop.

### 7. Enhanced Telemetry
Real-time performance metrics via WiFi:
```
Connect to: <ESP32_IP_ADDRESS>:23 (Telnet)
Commands: START, STOP, RESET, STATUS, CAL, HELP
```

### 8. Configurable Profiles
Switch between test and competition settings:
```cpp
PROFILE_TEST_MAZE      // Conservative for practice
PROFILE_COMPETITION    // Aggressive for winning
PROFILE_DEBUG          // Slow for debugging
```

### 9. Emergency Stop
Long press button (2 seconds) to stop immediately.

## Competition Day Checklist

### Before Competition
- [ ] Fully charge battery (8.4V)
- [ ] Clean sensor array lens
- [ ] Verify all connections tight
- [ ] Test motors in both directions
- [ ] Backup code to USB drive

### At Venue (30 min before run)
- [ ] Connect to venue WiFi (update SSID/password if needed)
- [ ] Recalibrate sensors in competition lighting
- [ ] Change profile to `PROFILE_COMPETITION`
- [ ] Test run on practice maze
- [ ] Verify line end detection works
- [ ] Check encoder counts per segment

### Just Before Run 1
- [ ] Fresh battery installed
- [ ] Robot positioned at START
- [ ] Verify sensor array 3-5mm from ground
- [ ] Clear path memory (send "RESET" via WiFi if needed)

### Between Run 1 and Run 2
- [ ] Verify optimization completed
- [ ] Check optimized path makes sense
- [ ] Battery voltage still good (> 7.0V)

## Tuning Guide

### PID Tuning (if needed)
Current values: Kp=45.0, Ki=0.5, Kd=25.0

**If robot oscillates:**
- Decrease Kp (try 35-40)
- Increase Kd (try 30-35)

**If robot responds slowly:**
- Increase Kp (try 50-55)
- Decrease Kd (try 20)

### Speed Tuning
Adjust in `src/Pins.h`:
```cpp
#define BASE_SPEED 150      // Normal line following
#define HIGH_SPEED 255      // Maximum for straights
#define TURN_SPEED 180      // During turns
```

### Junction Debounce
Adjust in `loadProfile()`:
```cpp
junctionDebounce = 300;  // ms between junction detections
```
- Smaller segments → increase (400-500ms)
- Larger segments → decrease (200-300ms)

## Troubleshooting

### Issue: Random turns on straight lines
**Cause:** False junction detection due to sensor overlap  
**Fix:** Verify only 3-4 sensors active on straight line (not 5+)

### Issue: Missing junctions
**Cause:** Debounce too long or speed too high  
**Fix:** Reduce debounce or add slower approach speed

### Issue: Robot loses line on turns
**Cause:** Turn speed too high or PID not tuned  
**Fix:** Reduce TURN_SPEED, increase Kp

### Issue: Line end not detected
**Cause:** Sensors not all seeing black simultaneously  
**Fix:** Adjust sensor height, verify LINE_END_CONFIRM_TIME

### Issue: Optimization removes needed moves
**Cause:** Path optimizer too aggressive  
**Fix:** Check path patterns in optimization rules

### Issue: WiFi not connecting
**Cause:** Wrong credentials or channel conflict  
**Fix:** Update SSID/PASSWORD, use 2.4GHz network

## Performance Metrics

| Metric | Target | Excellent |
|--------|--------|-----------|
| **Junction Detection Accuracy** | > 90% | > 98% |
| **False Positive Rate** | < 5% | < 1% |
| **Line Following Error** | < ±2.0 | < ±1.0 |
| **Run 2 Speed Improvement** | > 20% | > 35% |
| **Maze Completion Rate** | > 95% | 100% |

## Known Limitations

1. **ADC2 unavailable** when WiFi active (using ADC1 only)
2. **Maximum 100 junctions** (MAX_PATH_LENGTH limit)
3. **3.3V sensor power** reduces IR intensity vs 5V
4. **Encoder-based distance** affected by wheel slip
5. **WiFi telemetry** adds slight processing delay

## Future Enhancements

- [ ] Gyroscope for precise angle tracking
- [ ] Advanced path prediction with machine learning
- [ ] Multi-maze memory (store multiple solutions)
- [ ] Bluetooth control as WiFi alternative
- [ ] Battery level display and low-voltage warning
- [ ] Automatic speed profiling based on maze characteristics

## Credits

Developed for IIT Bombay Mesmerize Line Maze Competition  
**Repository:** github.com/Kk-SiNg/LMS_using_5_channel_sensor_array  
**Version:** 2.0 (8-Channel Edition)

## License

MIT License - Feel free to use and modify for your competition!

---

## Quick Start Summary

```bash
1. Wire RLS-08 per pinout above
2. Upload Test_RLS08_Basic to verify sensors
3. Upload Test_RLS08_Calibration and adjust pots
4. Upload main code
5. Calibrate on competition surface
6. Run 1: Map maze with LSRB
7. Run 2: Solve with optimized path
8. Win! 🏆
```

**Good luck at the competition!** 🚀