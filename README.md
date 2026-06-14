# 🌊 Autonomous Submarine Glider — Edge AI & Telemetry Stack

> **Lake Victoria Water Quality Monitor** — Autonomous underwater glider with on-board TinyML anomaly detection, dual-ESP32 LoRa telemetry, and real-time Azure IoT dashboard. Built for environmental monitoring in freshwater conditions where satellite telemetry fails and cloud inference is impractical.

**My Role:** Lead — Intelligence & Communication Layers (ESP32 firmware, TinyML pipeline, LoRa bridge, Azure dashboard)
**Team:** 3 members · Hardware/STM32 control by teammate · KCL MSc Capstone · **Individual Distinction**

---

## What I Built

```
[ESP32-S3 Intelligence Hub]
    ├── 4× water quality sensors (pH, DO, EC, Temp) @ 10 Hz → circular FIFO
    ├── TinyML DNN (30→32→16→4) — INT8 quantised, 5.64 KB, ~1 ms/inference
    │     └── Classes: NORMAL / ENV-ANOMALY / SENSOR-FAULT / NAV-DRIFT
    ├── Rule-based watchdog (fallback when confidence < 85%)
    └── STATE_EMERGENCY → STM32 via UART (buoyancy release trigger)

[Dual-ESP32 LoRa Bridge]
    ├── Submerged node (SX1276, 868 MHz, 14 dBm) — surface-burst on ascent > 0.2 m
    ├── Shore relay → Serial-to-USB → Python gateway
    └── Microsoft Azure IoT Hub → real-time dashboard

[Dual-Battery Isolation]
    ├── Main pack → actuator + motor (dirty domain)
    └── Lipstick cell (5V, 3000 mAh) → ESP32 + LoRa + sensors (clean domain)
          ~48h operating time under duty-cycled inference
```

---

## Why These Design Decisions

**LoRa over satellite** — Persistent tropical cloud cover degrades satellite reliability over Lake Victoria. LoRa at 868 MHz is cost-independent and cloud-independent. At 1–2 m depth in freshwater, RF attenuation is low enough for submerged transmission — eliminating the energy cost of full surface breach.

**TinyML on-device over cloud inference** — AUVs face extreme data scarcity and power constraints. Transmitting raw sensor streams to cloud is incompatible with a 48h battery budget. INT8 quantisation shrinks the model 4× (22.56 KB → 5.64 KB) with 3× inference speedup and only marginal accuracy loss.

**Hybrid fallback (TinyML + rule-based watchdog)** — A pure TinyML approach risks false-negatives under INT8 precision limits. A pure rule-based approach misses complex multi-sensor anomaly patterns. The hybrid distributes failure domains: neither layer alone can silence the emergency path.

**Galvanic isolation** — Propulsion EMI contaminates analog sensor readings. Separate power domains prevent fault propagation from the intelligence layer to vehicle control.

---

## Performance

| Metric | Value |
|--------|-------|
| TinyML model size (INT8) | **5.64 KB** |
| Test accuracy | **96.03%** |
| Macro F1 | **0.7967** |
| Inference latency | **~1 ms** |
| LoRa PDR @ 100 m | **94.2%** |
| Average RSSI (submerged) | **−105 dBm** |
| End-to-end detection → Azure dashboard | **1.85 s** |
| Emergency response (anomaly → STATE_EMERGENCY) | **1.2 s** |
| Logic layer operating time | **~48 h** |

---

## Technical Stack

| Layer | Tech |
|-------|------|
| Edge AI | TFLite Micro (ESP32), INT8 post-training quantisation, 30-32-16-4 DNN |
| Training data | InWaterSense + AUV navigation dataset, custom augmentation |
| Communication | SX1276 LoRa 868 MHz, dual-ESP32 bridge, custom packet protocol |
| Cloud | Microsoft Azure IoT Hub, Python gateway |
| Firmware | ESP32-S3 (Arduino/ESP-IDF), STM32 (bare-metal C, UART/SPI) |
| Dashboard | React/TypeScript, Web Serial API, 60+ FPS real-time visualisation |

---

## Repository Structure

```
submarine_dev/
├── simulation/          # Glider trajectory & gateway simulations
├── lora_base_station/   # Shore relay sketch (ESP32)
├── tflite_test/         # TFLite Micro inference validation
└── training/            # Model training, quantisation, INT8 export

submarine-test/          # STM32CubeIDE firmware (teammate — control/sensing)
```

---

## Key Engineering Trade-offs

**INT8 quantisation risk (FMEA RPN 160)** — The highest-priority system risk. INT8 reduces precision, and open-water validation data for rapid pressure transients doesn't exist yet. Mitigated by the hybrid fallback, but not eliminated. Future work: open-water field deployment in Lake Victoria to build real anomaly training data.

**Circular reuse strategy** — The 17.5 kg steel pressure hull was reused from the 2025 platform. The intelligence layer is not optional here — without reliable anomaly detection and telemetry, hull loss becomes an environmental liability (17.5 kg of permanent marine debris). TinyML + LoRa is the sustainability enabler, not just a feature.

---

## Assessment

**Individual Distinction** · KCL MSc Robotics Capstone 2026

*Assessed on: Project impact (70) · Communication (70) · Project management (75) · Sustainability (75) · LCA (70) · FMEA (70)*
