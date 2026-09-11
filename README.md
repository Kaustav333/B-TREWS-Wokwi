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
│  │  │  STATE 1: CAUTION                                    │  │   │
│  │  │    → Yellow LED ON, Relay CLOSED                     │  │   │
│  │  │                                                      │  │   │
│  │  │  STATE 2: WARNING                                    │  │   │
│  │  │    → Amber/Orange LED ON, CAN Bus Warning            │  │   │
│  │  │                                                      │  │   │
│  │  │  STATE 3: CRITICAL                                   │  │   │
│  │  │    → Red LED ON, Buzzer ALARM, RELAY TRIPS (Load OFF)│  │   │
│  │  └─────────────────────────────────────────────────────┘  │   │
│  └──────────────────────────────────────────────────────────┘   │
│                                                                  │
│                    ACTUATION LAYER                               │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌─────────┐ │
│  │ 🟢 Green │ │ 🟡 Yellow│ │ 🟠 Amber │ │ 🔴 Red   │ │ 10A SSR │ │
│  │ LED      │ │ LED      │ │ LED      │ │ LED      │ │ Relay   │ │
│  │ (GPIO 13)│ │ (GPIO 12)│ │ (GPIO 14)│ │ (GPIO 21)│ │(GPIO 48)│ │
│  └──────────┘ └──────────┘ └──────────┘ └──────────┘ └─────────┘ │
└──────────────────────────────────────────────────────────────────┘
```

---

## 📡 Sensor Specifications & Thresholds

Based on the **Final Optimized COMSOL Reference Case** and project-matched triggers:

| State | Wokwi Condition | Project Basis |
|:---|:---|:---|
| **NORMAL** | `dT/dt <= 0.041 C/s` AND `dP/dt <= 0.041317 hPa/s` AND `VOC < 0.05` | Matches mild thermal Case 1, lowest pressure-rate sweep reference, and no VOC arrival. |
| **CAUTION** | `dT/dt > 0.041` and `< 0.408 C/s` OR `dP/dt > 0.041317` and `< 0.69313 hPa/s` OR `VOC >= 0.05` | Between mild and strong modeled behavior; VOC arrival detected. |
| **WARNING** | `dT/dt >= 0.408` and `<= 1.2 C/s` OR `dP/dt >= 0.69313` and `< 2.5 hPa/s` | Anchored to thermal Case 2, final optimized dP/dt, and project critical limits. |
| **CRITICAL** | `dT/dt > 1.2 C/s` OR `dP/dt >= 2.5 hPa/s` | Uses the actual project candidate critical triggers. |

---

## 🔬 COMSOL Multiphysics Integration

The firmware includes a **COMSOL Multiphysics playback engine** that replays thermal runaway simulation data through the ESP32-S3 state machine. The time-series dataset in `src/main.cpp` can be replaced with actual COMSOL CFD & Thermal output values:

```cpp
COMSOL_Point comsol_profile[] = {
  // { Time(s), Pressure(hPa), Temp(C), VOC (mol/m^3) }
  { 0.0,  1013.25, 20.00, 0.00 }, // NORMAL
  { 10.0, 1014.44, 21.20, 0.05 }, // CAUTION
  { 15.0, 1017.91, 23.24, 0.10 }, // WARNING
  { 20.0, 1058.01, 42.70, 1.00 }, // CRITICAL TRIP
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
| Yellow LED + 220Ω | `wokwi-led` + `wokwi-resistor` | STATE 1: Caution indicator |
| Amber LED + 220Ω | `wokwi-led` + `wokwi-resistor` | STATE 2: Warning indicator |
| Red LED + 220Ω | `wokwi-led` + `wokwi-resistor` | STATE 3: Critical Trip indicator |
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
