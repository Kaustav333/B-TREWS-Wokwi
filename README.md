# B-TREWS: Battery-Mounted Thermal Runaway Early-Warning System

> **Capstone Phase 1 — Task 1.2: Embedded Emulation via Wokwi ESP32-S3 Simulator**

## 🔋 Project Overview

**B-TREWS** is a real-time, multi-sensor safety system designed to detect Li-ion battery thermal runaway **5–15 minutes before catastrophic failure**. It uses a combination of VOC off-gas detection, micro-cavity pressure monitoring, and busbar thermal acceleration sensing to provide early warnings and automatically disconnect the battery pack load via a solid-state relay.

This repository contains the **Wokwi embedded simulation** of the B-TREWS system running on an **ESP32-S3 microcontroller**, with COMSOL Multiphysics thermal runaway data playback.

---

## 🏗️ System Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                    B-TREWS SENSING LAYER                        │
│                                                                  │
│  ┌─────────────┐  ┌─────────────────┐  ┌─────────────────────┐  │
│  │ MQ-135 VOC  │  │ Bosch BMP280    │  │ 10k NTC Busbar     │  │
│  │ Gas Sensor  │  │ Pressure/Temp   │  │ Thermal Probe      │  │
│  │ (GPIO 7)    │  │ (I2C: SDA=4,    │  │ (GPIO 8)           │  │
│  │             │  │  SCL=5)         │  │                     │  │
│  └──────┬──────┘  └───────┬─────────┘  └──────────┬──────────┘  │
│         │                 │                        │             │
│         ▼                 ▼                        ▼             │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │              ESP32-S3 PROCESSING ENGINE                   │   │
│  │                                                           │   │
│  │  ┌─────────────────────────────────────────────────────┐  │   │
│  │  │          DERIVATIVE CALCULATION ENGINE               │  │   │
│  │  │  dP/dt = (P_current - P_previous) / Δt              │  │   │
│  │  │  dT/dt = (T_current - T_previous) / Δt              │  │   │
│  │  └─────────────────────────────────────────────────────┘  │   │
│  │                          │                                │   │
│  │                          ▼                                │   │
│  │  ┌─────────────────────────────────────────────────────┐  │   │
│  │  │           STATE MACHINE DECISION ENGINE              │  │   │
│  │  │                                                      │  │   │
│  │  │  STATE 0: NORMAL                                     │  │   │
│  │  │    → Green LED ON, Relay CLOSED, Load Active         │  │   │
│  │  │                                                      │  │   │
│  │  │  STATE 1: WARNING (VOC > 2500 OR dP/dt > 2.5 hPa/s) │  │   │
│  │  │    → Amber LED ON, CAN Bus Warning Frame TX          │  │   │
│  │  │                                                      │  │   │
│  │  │  STATE 2: CRITICAL (dT/dt > 1.2°C/s OR T > 60°C)    │  │   │
│  │  │    → Red LED ON, Buzzer ALARM, RELAY TRIPS (Load OFF)│  │   │
│  │  └─────────────────────────────────────────────────────┘  │   │
│  └──────────────────────────────────────────────────────────┘   │
│                                                                  │
│                    ACTUATION LAYER                               │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌───────────────┐   │
│  │ 🟢 Green │  │ 🟠 Amber │  │ 🔴 Red   │  │ 10A SSR Relay │   │
│  │ LED      │  │ LED      │  │ LED      │  │ + Load Lamp   │   │
│  │ (GPIO 13)│  │ (GPIO 14)│  │ (GPIO 21)│  │ (GPIO 48)     │   │
│  └──────────┘  └──────────┘  └──────────┘  └───────────────┘   │
└──────────────────────────────────────────────────────────────────┘
```

---

## 📡 Sensor Specifications & Thresholds

| Sensor | Physical Phenomenon | Detection Threshold | Response |
|:---|:---|:---|:---|
| **MQ-135 VOC Gas Sensor** | SEI decomposition: DEC/EMC volatile organic solvent venting (<2s latency) | `VOC ADC > 2500` | Escalates to **STATE 1 WARNING** |
| **Bosch BMP280** | Micro-cavity convective pressure build-up above relief vent | `dP/dt > 2.5 hPa/s` | Sends CAN Warning Frame (`0x18FF0100`) |
| **10k NTC Busbar Probe** | Exothermic thermal acceleration at negative terminal | `dT/dt > 1.2°C/s` OR `T > 60°C` | Escalates to **STATE 2 CRITICAL TRIP** |

---

## 🔬 COMSOL Multiphysics Integration

The firmware includes a **COMSOL Multiphysics playback engine** that replays thermal runaway simulation data through the ESP32-S3 state machine. The time-series dataset in `src/main.cpp` can be replaced with actual COMSOL CFD & Thermal output values:

```cpp
COMSOL_Point comsol_profile[] = {
  // { Time(s), Pressure(hPa), Temp(°C), VOC_Gas(ADC) }
  { 0.0,  1013.25, 25.0, 400  },  // Normal operation
  { 20.0, 1018.80, 28.5, 2850 },  // STAGE 1: Off-gas spike -> WARNING
  { 45.0, 1026.10, 62.4, 3950 },  // STAGE 2: Thermal trip -> RELAY DISCONNECT
};
```

### Mode Switch
```cpp
#define USE_COMSOL_PLAYBACK  true   // Auto-play COMSOL dataset
#define USE_COMSOL_PLAYBACK  false  // Manual potentiometer & BMP280 control
```

---

## 🛠️ Hardware Components (Wokwi Simulation)

| Component | Wokwi Part | Function |
|:---|:---|:---|
| ESP32-S3-DevKitC-1 | `board-esp32-s3-devkitc-1` | Main microcontroller |
| Bosch BMP280 | `wokwi-bmp280` | Pressure & temperature sensor (I2C) |
| MQ-135 Gas Sensor | `wokwi-potentiometer` (analog sim) | VOC off-gas detection |
| 10k NTC Thermistor | `wokwi-potentiometer` (analog sim) | Busbar temperature probe |
| Green LED + 220Ω | `wokwi-led` + `wokwi-resistor` | STATE 0: Normal indicator |
| Amber LED + 220Ω | `wokwi-led` + `wokwi-resistor` | STATE 1: Warning indicator |
| Red LED + 220Ω | `wokwi-led` + `wokwi-resistor` | STATE 2: Critical Trip indicator |
| 10A Solid State Relay | `wokwi-relay` | Battery load disconnect switch |
| Piezo Buzzer | `wokwi-buzzer` | Emergency audible alarm |
| Blue LED + 220Ω | `wokwi-led` + `wokwi-resistor` | Battery pack load lamp |

---

## 🚀 How to Run

### Option A: Run Locally in VS Code (Recommended)

**Prerequisites:**
- [VS Code](https://code.visualstudio.com/)
- [PlatformIO IDE Extension](https://platformio.org/install/ide?install=vscode)
- [Wokwi Simulator Extension](https://marketplace.visualstudio.com/items?itemName=wokwi.wokwi-vscode)
- [Free Wokwi License](https://wokwi.com/dashboard/ci) (activate via `Ctrl+Shift+P` → `Wokwi: Request a New License`)

**Steps:**
1. Clone this repository:
   ```bash
   git clone https://github.com/Kaustav333/B-TREWS-Wokwi.git
   ```
2. Open the folder in VS Code:
   ```
   File → Open Folder → B-TREWS-Wokwi
   ```
3. Build the firmware:
   ```
   Ctrl+Shift+P → PlatformIO: Build
   ```
4. Start the simulation:
   ```
   Ctrl+Shift+P → Wokwi: Start Simulator
   ```
5. Press the **green Play (▶️)** button in the Wokwi Diagram Editor.

### Option B: Run on Wokwi Web

1. Go to [wokwi.com](https://wokwi.com) → Create new ESP32-S3 project.
2. Copy `diagram.json` content into the `diagram.json` tab.
3. Copy `src/main.cpp` content into the `sketch.ino` tab.
4. Add libraries: `Adafruit BMP280 Library` and `Adafruit Unified Sensor`.
5. Click **Play (▶️)**.

---

## 🎬 Simulation Timeline

| Time | State | Visual Output |
|:---|:---|:---|
| **0s – 15s** | STATE 0: Normal | 🟢 Green LED ON, Blue Load Lamp ON, Buzzer Silent |
| **15s – 35s** | STATE 1: Warning | 🟠 Amber LED ON, CAN Warning Frames on Serial Monitor |
| **35s onwards** | STATE 2: Critical Trip | 🔴 Red LED ON, Buzzer ALARM, Blue Lamp OFF (Relay Tripped) |

---

## 📁 Project Structure

```
B-TREWS-Wokwi/
├── src/
│   └── main.cpp          # ESP32-S3 firmware (state machine + COMSOL engine)
├── diagram.json          # Wokwi circuit diagram (all components & wiring)
├── platformio.ini        # PlatformIO build configuration
├── wokwi.toml            # Wokwi VS Code simulator configuration
└── README.md             # This file
```

---

## 👥 Team

**Capstone Project — Phase 1**

- Kaustav Kalita

---

## 📄 License

This project is developed as part of an academic capstone project.
