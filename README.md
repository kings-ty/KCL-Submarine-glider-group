# KCL Submarine Glider Group

Autonomous underwater glider project for **water-quality monitoring in Lake Victoria** using a circular reuse strategy, TinyML edge intelligence, and LoRa-based telemetry.

This README is written in English from your final report content and assessment feedback.

## Chapter 1. Introduction

### 1.1 Problem Statement and Societal Relevance
Lake Victoria supports over 42 million livelihoods but is increasingly threatened by untreated wastewater and agricultural runoff.  
The project addresses this challenge through a low-cost, low-impact autonomous monitoring platform aligned with **SDG 6 (Clean Water)** and **SDG 14 (Life Below Water)**.

### 1.2 Pre-study and Design Validation
Early Python simulations were used to validate buoyancy engine parameters and confirm the feasibility of sawtooth trajectory operation for the glider mass profile.

### 1.3 Individual Contribution and Technical Justification
The core contribution focused on the **intelligence and communication layers**:
- Multi-board distributed architecture (ESP32 for AI/comms + STM32 for control/sensing)
- Dual-ESP32 LoRa bridge for robust, low-cost telemetry
- TinyML anomaly detection for on-board prioritisation of critical events

This software stack was essential for reliable recovery, enabling circular reuse of the 17.5 kg steel pressure hull from the 2025 platform.

### 1.4 Reflective Analysis Framework
The report reflects on system impact through project management, values/stakeholder analysis, sustainability ethics, LCA, and reliability (FMEA).  
A central finding is that TinyML+LoRa both enables reuse and introduces top-priority risk requiring hybrid fail-safe logic.

## Chapter 2. Project Management, Values Thinking, and Stakeholder Dialogue

### 2.1 Project Management and Technical Leadership
- Initial planning: WBS + Gantt for deliverables and critical path
- Mid-project pivot: Kanban Agile due to procurement delays and dependency uncertainty
- Team enablement: strength-based delegation, preconfigured environments, and SOP-driven collaboration

### 2.2 Values Thinking: Efficiency vs Responsibility
Low-cost rapid prototyping (PLA parts) improved feasibility, but ethical trade-offs were explicitly considered (microplastic risk, material lifecycle limits).  
Prototype decisions prioritized practical deployment while planning transition to more sustainable production materials.

### 2.3 Stakeholder Dialogue
The design balanced:
- **Fishers / marine biologists**: strong in-situ sensing coverage
- **Regulators**: habitat safety and recoverability requirements

Fail-to-float and LoRa-based recovery strategy established a non-intrusive middle-ground solution.

## Chapter 3. Sustainability and Ethics

### 3.1 Doughnut Framing and Systems Perspective
The project was mapped to environmental boundaries and social foundations, linking biosphere protection with equitable access to water-quality intelligence.

### 3.2 Causal-Loop Insight
- **Reinforcing loop (R1):** better intelligence → better recovery → better asset reuse → lower per-mission impact
- **Balancing loop (B1):** higher data frequency → higher energy demand → battery constraints

### 3.3 Circular Economy Strategy
Hybrid material strategy:
- Reused high-mass steel hull (long-life structural asset)
- Replaceable lightweight components for adaptation
- Software intelligence as a life-extension multiplier for physical assets

### 3.4 Reflective Ethics and SDG Trade-offs
Key reflections included SDG synergies/trade-offs, data transparency vs operational security, and governance needs for responsible open environmental data.

### 3.5 Professional Ethics
Engineering decisions were framed around integrity, environmental respect, rigor, and communication/leadership principles.

## Chapter 4. Life Cycle Assessment (LCA)

### 4.1 Method and Circular Baseline
ISO 14040-style framing was applied to compare:
- Circular reuse strategy (2026 upgrade)
- Full-manufacture baseline
- Alternative composite-hull scenario

Functional unit: one successful 48-hour monitoring mission, amortised across a 50-mission design life.

### 4.2 Key LCA Interpretation
- Circular reuse significantly reduced total impact versus full-manufacture assumptions
- Intelligence layer (TinyML + LoRa) acted as a sustainability enabler by improving recovery confidence
- Allocation-method choice (cut-off vs system expansion) materially affects impact narratives and must be transparently acknowledged

## Chapter 5. Reliability, FMEA, and Architecture Optimization

### 5.1 FMEA Method
Risk prioritisation followed RPN = Severity × Occurrence × Detection.  
Critical risks were linked to mitigation evidence from bench and tank validation.

### 5.2 High-Priority Risk Reflection
The top intelligence risk was TinyML false-negative under constrained edge conditions (INT8 + memory limits).  
A hybrid architecture (TinyML + rule-based watchdog) was retained to distribute failure domains instead of shifting risk to a single cloud-dependent path.

### 5.3 Power and Communication Reliability
Dual-battery isolation and context-aware communication strategy were used to reduce power noise, preserve mission endurance, and improve link robustness.

## Chapter 6. Conclusion

This project demonstrates that embedding edge intelligence into legacy hardware can be a practical **sustainability strategy**, not only a technical upgrade.  
It enabled circular reuse, improved mission-level observability, and supported real-world environmental stewardship goals while maintaining a critical view of unresolved risk and data limitations.

## Quantitative Highlights

- TinyML INT8 model size: **5.64 KB**
- Classification accuracy: **96.03%**
- Macro F1: **0.7967**
- LoRa PDR at 100 m: **94.2%**
- End-to-end dashboard latency: **~1.85 s**
- Intelligence high-priority FMEA risk: **RPN 160**

## Assessment Summary (Provided)

- **Project impact:** 70  
  Excellent linkage between technical design and societal impact in Lake Victoria.
- **Communication:** 70  
  Strong chapter flow, professional writing, and consistent academic tone.
- **Project management / values:** 75  
  Strong reflection on Gantt→Kanban transition and stakeholder/value trade-offs.
- **Sustainability:** 75  
  Strong systems framing (Doughnut + CLD) and SDG interdependency analysis.
- **LCA:** 70  
  Clear circular strategy, quantitative scenario comparison, and allocation-method critique.
- **FMEA:** 70  
  Rigorous risk prioritisation with mitigation evidence and architecture linkage.

## Repository Structure

- `submarine-test/` — STM32CubeIDE firmware project
- `submarine_dev/` — TinyML training, quantization, simulation, and ESP32 integration
  - `simulation/` — glider and gateway simulations
  - `lora_base_station/` — LoRa base-station sketch
  - `tflite_test/` — TFLite Micro inference test
- Root Python scripts — serial/comms/simulation support utilities

## Reference Images

![test-ucl](https://github.com/user-attachments/assets/fa977f7b-3119-4786-85b9-683699786343)
![lipstick-battery](https://github.com/user-attachments/assets/ebf3c85a-7f7b-41c1-b9d1-114b3c05451d)
![monitoring](https://github.com/user-attachments/assets/d98f4bd2-8ca5-4738-a45c-ae644dd94472)
![WhatsApp Image 2026-04-03 at 17 12 19](https://github.com/user-attachments/assets/746e3bb6-1338-476e-b979-9734b06d545d)
