# Dual-Layer Automotive BMS with Edge-AI Thermal Runaway Prediction

> **4S LiFePO4 Battery Management System** | STM32F407 Safety Layer + ESP32-S3 Intelligence Layer | CAN Bus | TFLite Micro | V2X ESP-NOW | Cloud Dashboard

---

## Overview

A two-microcontroller BMS designed for automotive-grade battery packs, featuring real-time thermal runaway prediction using a quantized 1D CNN running directly on the edge — no cloud dependency for safety-critical decisions.

Built to address a gap in published BMS research: no existing paper combines LiFePO4 chemistry (flat voltage curve makes thermal analysis critical), aged cells, and edge AI deployment at <70KB model size on a real MCU.

**Project context:** Designed for Volvo "Connected Safety" use case | SRM Institute of Science and Technology, Batch 2027

---

## System Architecture

```
┌─────────────────────────────┐         ┌──────────────────────────────────┐
│   STM32F407 — SAFETY LAYER  │         │  ESP32-S3 — INTELLIGENCE LAYER   │
│                             │         │                                  │
│  • 8x DS18B20 temp sensors  │  CAN    │  • MCP2515 SPI-to-CAN receiver   │
│  • 4x ADC voltage taps      │ 125kbps │  • 1D CNN TFLite Micro inference  │
│  • 16x oversampled ADC      │ ──────► │  • 3-tier gradient analysis      │
│  • IIR voltage filter       │         │  • Thermal gate + persist filter  │
│  • Outlier rejection filter │         │  • OLED SPI display (4 pages)    │
│  • CAN TX: 4 frames         │         │  • Ubidots cloud dashboard       │
│  • LED + Buzzer alerts      │         │  • V2X ESP-NOW broadcast alerts  │
└─────────────────────────────┘         └──────────────────────────────────┘
         │                                            │
         └──────── 4S LiFePO4 Pack ──────────────────┘
              32700 cells | 12.8V nominal | 6Ah
```

---

## Repository Structure

```
automotive-bms-edge-ai/
│
├── stm32_safety_layer/
│   └── main.c                    # STM32F407 firmware (STM32CubeIDE)
│
├── esp32_intelligence_layer/
│   ├── esp32_bms_v29_edge_ai.ino # ESP32-S3 firmware (Arduino IDE)
│   └── bms_cnn_model.h           # INT8 quantized TFLite model (29,616 bytes)
│
└── README.md
```

---

## Hardware

| Component | Part | Role |
|-----------|------|------|
| Safety MCU | STM32F407G-DISC1 | Voltage/temp measurement, CAN TX |
| Intelligence MCU | ESP32-S3 N8R8 | Edge-AI inference, cloud, V2X |
| Battery | 4S LiFePO4 32700 (6Ah) | 12.8V nominal pack |
| BMS Protection | TDT 4S 10A | Hardware OVP/UVP/OCP |
| Temp Sensors | 8x DS18B20 | 6 cell body + 2 terminal |
| CAN Transceiver | MCP2515 + TJA1050 | SPI-to-CAN bridge |
| Display | SSD1306 OLED (SPI) | 4-page live dashboard |
| Regulator | LM2596 | 5V from pack |

**Total BOM cost: ~₹7,500** (vs ₹75,000+ commercial BMS with AI)

---

## Key Technical Features

### 1. Three-Tier Thermal Gradient Analysis (Project Novelty)

Most BMS systems use a single temperature threshold. This system implements three independent gradient analyses:

| Tier | Measurement | WARN | DANGER |
|------|------------|------|--------|
| **Intra-cell** | \|T_left − T_right\| per cell | >2°C | >5°C |
| **Terminal** | \|T_positive − T_negative\| | >3°C | >8°C |
| **Inter-cell** | max(T_body) − min(T_body) | >3°C | >6°C |

This catches hotspots that a single-sensor or average-temperature approach would miss entirely.

### 2. Edge-AI: 1D CNN Thermal Runaway Prediction

- **Architecture:** Conv1D(32) → Conv1D(64) → Conv1D(32) → GAP → Dense(32) → Dense(4)
- **Input:** 30 timesteps × 14 features (cell voltages + temps + gradients)
- **Output:** 4 classes — NORMAL / WARNING / HIGH / DANGER
- **Model size:** 29,616 bytes (INT8 quantized)
- **Inference time:** 27–28ms on ESP32-S3
- **Arena usage:** 7,136 / 32,768 bytes

**Why 1D CNN over LSTM?** LSTM requires `SELECT_TF_OPS` which is unsupported on TFLite Micro. 1D CNN achieves equivalent temporal pattern recognition within the standard op set.

### 3. Post-Inference Correction Pipeline

Raw CNN output passes through three stages to eliminate false positives:

```
Raw CNN Score
    │
    ▼
Thermal Gate — if thermals are flat (no dT/dt, no inter-gradient),
    │           discount AI risk by 80%. Prevents voltage-only
    │           imbalance from triggering false thermal alarm.
    ▼
Persistence Filter — require 5 consecutive WARNING+ frames before
    │                 escalating. Kills startup transients.
    ▼
EMA Smoothing — α=0.3 (30% new, 70% previous). Frame-to-frame
    │            noise elimination.
    ▼
Combined Risk = max(threshold_score, corrected_ai_score)
```

### 4. V2X ESP-NOW Broadcast

When `combined_risk ≥ 30%`, the system broadcasts a 32-byte alert packet to all nearby devices (broadcast MAC `FF:FF:FF:FF:FF:FF`) using ESP-NOW — no WiFi/internet required. Range ~200m line-of-sight, latency <5ms.

### 5. STM32 ADC — PA1 Auto-Calibration

PA1 (Cell 2/3 tap) exhibited boot-to-boot voltage drift. Solution: at startup, compute expected PA1 value as midpoint of PA2 and PC1 (valid for balanced LiFePO4 cells), derive correction factor dynamically. Clamped to [0.7, 1.1] range.

---

## CAN Bus Protocol

**Baud rate:** 125 kbps | **APB1:** 6.25 MHz | Prescaler=5, BS1=8TQ, BS2=1TQ

| Frame ID | Payload | Description |
|----------|---------|-------------|
| `0x100` | uint16 ×4 (mV, big-endian) | Cell voltages C1–C4 |
| `0x101` | int16 ×4 (°C ×10) | Cell1 L/R, Cell2 L/R temps |
| `0x102` | int16 ×4 (°C ×10) | Cell3 L/R, Pos/Neg terminal temps |
| `0x103` | PackV, MinV, Status, LoopCount | Pack summary + status byte |

---

## Sensor Mapping

```
Index  Location        Role
S0     Cell 3 Right    Body temp
S1     Cell 1 Left     Body temp
S2     Cell 2 Left     Body temp
S3     Cell 2 Right    Body temp
S4     Negative Term.  Terminal temp
S5     Cell 3 Left     Body temp
S6     Cell 1 Right    Body temp
S7     Positive Term.  Terminal temp
```

---

## Fault Test Results (Hair Dryer on Cell 1)

| Time | Max Temp | dT/dt | Inter-gradient | Combined Risk | TTC |
|------|----------|-------|----------------|---------------|-----|
| Baseline | 23.1°C | — | — | 5% | — |
| t=3s | 25.5°C | +0.954°C/min | — | AI=DANGER 99.6% | 30.9 min |
| t=9s | 28.1°C | +2.138°C/min | 3.83°C [WARN] | 48% | — |
| t=12s | 31.0°C | +3.287°C/min | 6.64°C [DANGER] | 55% | **7.3 min** |

**Result: 10–15 minutes early warning before thermal runaway threshold.**

---

## Cloud Dashboard (Ubidots)

10 variables streamed every 10s:

`pack_voltage` · `v_imbalance` · `max_temp` · `dt_dt` · `intra_gradient` · `term_gradient` · `inter_gradient` · `combined_risk` · `ai_risk_corrected` · `time_to_critical`

---

## Build Environment

| Layer | Tool | Board Config |
|-------|------|-------------|
| STM32 | STM32CubeIDE | STM32F407G-DISC1, HSI→PLL→25MHz |
| ESP32 | Arduino IDE | ESP32S3 Dev Module, USB CDC On Boot: Enabled, PSRAM: OPI PSRAM |

**ESP32 Libraries:** `mcp_can`, `Adafruit_SSD1306`, `Adafruit_GFX`, `Chirale_TensorFlowLite`, `esp_now`

---

## Team

Mohammed Ayaan · Aditya · Dhanush  
Course: 21ECP302L — SRM Institute of Science and Technology, Kattankulathur
