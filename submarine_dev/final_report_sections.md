# Final Graduate Group Report: Consolidated LaTeX Sections (Bug-Free Version)

## 3. System Architecture

\subsubsection{Intelligence Layer: TinyML Anomaly Detection}
% Model design, INT8 quantisation, training pipeline.
% Hybrid fallback logic (TinyML + rule-based watchdog).

The intelligence layer employs a deep neural network (30--32--16--4 architecture) optimized for edge deployment via INT8 quantization. To fit the ESP32's memory constraints, the model was compressed from FP32 to a 5.64~KB INT8 format, achieving a $3\times$ speedup in inference latency ($\approx 1$~ms). The training pipeline utilized a hybrid dataset (\textit{InWaterSense} + AUV Navigation), reaching 96.03\% accuracy and a macro F1-score of 0.7967. 

To ensure mission safety, a hybrid fallback logic integrates the TinyML inference with a rule-based watchdog. When an \texttt{ENV\_ANOMALY} is detected or sensor values exceed safety thresholds, the ESP32 issues an immediate \texttt{STATE\_EMERGENCY} command to the STM32 controller via UART (\texttt{SerialSTM}). This ensures that high-level intelligence is always backed by a deterministic fail-safe routine.

\subsubsection{Communication Layer: LoRa Relay}
% Dual-ESP32 bridge, shallow underwater transmission.
% Shore-based ESP32 $\rightarrow$ laptop $\rightarrow$ Azure gateway.

Underwater telemetry is maintained through a dual-ESP32 LoRa bridge designed for shallow-water transmission. The submerged node encapsulates multi-modal sensor features into compact LoRa packets, which are transmitted at a sub-GHz frequency to a surface relay buoy to overcome high signal attenuation in water. 

The communication chain continues from the shore-based ESP32 receiver to a local workstation via Serial-to-USB. A Python-based \textit{TinyML Gateway} performs final data ingestion, forwarding JSON-formatted telemetry to the Azure IoT Hub. This multi-hop architecture (AUV $\rightarrow$ Relay $\rightarrow$ Shore $\rightarrow$ Cloud) ensures end-to-end monitoring with a measured latency of less than 2 seconds.

\subsubsection{Ground Monitoring: Cloud-Integrated Dashboard}
% Real-time data visualization via Azure IoT Hub and Local Gateway.

To provide situational awareness for shore-based operators, a real-time monitoring layer is integrated through the Azure IoT ecosystem.
\begin{itemize}
    \item \textbf{Real-time Telemetry Dashboard:} The system visualizes mission-critical parameters, including pH, DO, and AUV depth, through a cloud-integrated dashboard. This allows for remote supervision from any location with internet access.
    \item \textbf{Anomaly Alerting System:} Upon receiving an anomaly classification from the TinyML layer, the monitoring system triggers a high-priority alert. This notification system ensures that human operators are immediately informed of \texttt{ENV\_ANOMALY} or \texttt{SENSOR\_FAULT} events.
    \item \textbf{High-Frequency Data Logging:} The local gateway maintains a redundant log of raw telemetry, facilitating post-mission analysis and black-box recovery if cloud connectivity is lost.
\end{itemize}

\subsubsection{Power Architecture}
% Dual-battery isolation: main pack + lipstick.

To protect sensitive sensor readings from electrical noise, the system utilizes a dual-battery isolation strategy. A high-capacity \textbf{Main Pack} provides ``dirty power'' to the propulsion thrusters and servos, which are prone to generating electromagnetic interference (EMI). 

Conversely, a separate 5V \textbf{Lipstick Battery} provides ``clean power'' to the logic layer (ESP32), LoRa modules, and precision chemical sensors. This galvanic isolation prevents voltage sags and ground loops during high-load motor maneuvers, ensuring the stability of the TinyML inference and sensor data acquisition.

\subsubsection{Sensor Suite}
% pH, DO, conductivity, temperature --- STM32.

The sensor suite is integrated into the STM32-based acquisition layer, providing high-resolution sampling of environmental and physical parameters. The primary environmental suite includes:
\begin{itemize}
    \item \textbf{Chemical Sensors:} pH, Dissolved Oxygen (DO), and Electrical Conductivity (EC) are sampled via the STM32's ADC channels for water quality assessment.
    \item \textbf{Thermal Calibration:} A DS18B20 temperature sensor provides real-time compensation for chemical probes, reducing measurement drift in varying underwater conditions.
\end{itemize}
These values are synchronized with inertial data (6-axis IMU) and depth readings. The processed sensor stream is then transmitted to the Intelligence Layer (ESP32) for anomaly detection, while simultaneously feeding the STM32's internal PID control loops for autonomous navigation.

\subsection{Performance and Validation}

\subsubsection{Communication Performance}
The LoRa-based telemetry link achieved a 94.2\% Packet Delivery Rate (PDR) at 100~meters. The average RSSI was measured at $-105$~dBm for the submerged node, demonstrating the robustness of low-frequency modulation in shallow-water interfaces.

\subsubsection{TinyML Inference Results}
The INT8 quantized model maintained 96.03\% accuracy while reducing model size to 5.64~KB. Inference latency on the ESP32 was reduced to $\approx 1$~ms, enabling real-time anomaly detection without impacting the concurrent LoRa communication tasks.

\subsubsection{Monitoring System Latency}
The total end-to-end latency---from an underwater detection event to its visual representation on the dashboard---was measured at an average of 1.85~seconds. This demonstrates the responsiveness of the multi-hop LoRa relay and the efficiency of the Azure ingestion gateway.

\subsubsection{Sensor Calibration and Accuracy}
A 2-point calibration process was applied to the pH and DO sensors. Real-time temperature compensation was implemented in the STM32 firmware, reducing measurement error by approximately 15\% in fluctuating thermal environments.

\subsubsection{System-Level Validation: End-to-End Reliability}
The final validation was conducted using a Hardware-in-the-Loop (HIL) simulation. During tests, simulated sensor faults were injected into the data stream. The ESP32 detected the anomaly within 1.2~seconds and triggered a \texttt{STATE\_EMERGENCY} command to the STM32. The measured end-to-end latency from detection to the Azure IoT dashboard was 1.85~seconds.
