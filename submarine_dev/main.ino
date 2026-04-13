// /*
//  * Triple Sensor Test: EC (A0), DO (A1), pH (A2)
//  * Hardware: DFRobot Gravity Analog Sensors
//  */

// const int EC_PIN = A1;
// const int DO_PIN = A0;
// const int PH_PIN = A2; // pH 센서 추가
// const float VREF = 5.0;

// void setup() {
//   Serial.begin(115200);
//   Serial.println("==========================================================");
//   Serial.println("  Triple Sensor Monitoring: EC, DO, pH");
//   Serial.println("==========================================================");
// }

// void loop() {
//   // 1. 각 센서에서 값 읽기
//   int ecRaw = analogRead(EC_PIN);
//   int doRaw = analogRead(DO_PIN);
//   int phRaw = analogRead(PH_PIN);

//   // 2. 전압(Voltage)으로 변환
//   float ecVolt = ecRaw * VREF / 1024.0;
//   float doVolt = doRaw * VREF / 1024.0;
//   float phVolt = phRaw * VREF / 1024.0;

//   // 3. 시리얼 모니터 출력
//   Serial.print("EC: ");   Serial.print(ecVolt, 2); Serial.print("V | ");
//   Serial.print("DO: ");   Serial.print(doVolt, 2); Serial.print("V | ");
//   Serial.print("pH: ");   Serial.print(phVolt, 2); Serial.println("V");

//   // 데이터 가독성을 위해 1초마다 출력
//   delay(1000);
// }