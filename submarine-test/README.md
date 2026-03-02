# STM32F407VET6 LED Control Test

STM32F407VET6 보드를 사용한 LED 제어 테스트 프로젝트입니다.

## 하드웨어

- **MCU**: STM32F407VET6
- **보드**: STM32F407VET6 개발보드
- **프로그래머**: ST-Link V2

## 기능

### 내장 LED
- **PA6**: 내장 LED (D2) - Sink 모드 (LOW에서 켜짐)
- **PA7**: 내장 LED - Sink 모드 (LOW에서 켜짐)

### 외장 LED
- **PB1**: 외장 LED 제어 - Source 모드 (HIGH에서 켜짐)

### LED 동작
- PB1 LED가 1초마다 ON/OFF 깜빡임
- USB CDC를 통해 상태 메시지 출력

## 연결 방법

### 외장 LED 연결
```
LED 긴 다리(+) → 저항(220Ω) → PB1 핀
LED 짧은 다리(-) → GND
```

### USB CDC 통신
- USB 케이블을 통해 시리얼 통신
- Baud rate: 기본값 (보통 115200)
- 터미널 프로그램: PuTTY, CoolTerm 등

## 빌드 및 프로그래밍

1. **STM32CubeIDE에서 빌드**
   ```
   Project → Build Project (Ctrl+B)
   ```

2. **STM32CubeProgrammer로 프로그래밍**
   - Connect to ST-Link
   - Load .bin file from Debug 폴더
   - Start Programming

## 설정

### ST-Link 연결 설정
- **Port**: SWD
- **Speed**: 기본값 (4000 kHz)
- **Reset Mode**: Software Reset

### GPIO 설정
- **PA6, PA7**: Output Push-Pull, Sink 모드
- **PB0, PB1**: Output Push-Pull, Source 모드
- **클럭**: GPIOA, GPIOB, GPIOC 활성화

## 문제 해결

### LED가 깜빡이지 않는 경우
1. 연결 확인 (LED 극성, 저항값)
2. STM32 물리적 리셋
3. 전원 재공급
4. 다른 GPIO 핀 시도

### ST-Link 연결 문제
1. USB 케이블 확인
2. ST-Link 드라이버 설치
3. STM32CubeProgrammer 재시작

### USB CDC 인식 안 됨
- ST-Link는 디버깅/프로그래밍용
- 시리얼 통신은 별도 USB-Serial 어댑터 필요

## 코드 구조

```c
main.c
├── SystemClock_Config()    // 시스템 클럭 설정
├── MX_GPIO_Init()         // GPIO 초기화
└── main loop              // LED 제어 루프
    ├── PB0, PB1 강제 OFF
    ├── PB1 ON (1초)
    ├── PB1 OFF (1초)
    └── 상태 출력
```

## 개발 환경

- **IDE**: STM32CubeIDE
- **프로그래머**: STM32CubeProgrammer
- **HAL 라이브러리**: STM32F4xx HAL
- **USB CDC**: Virtual COM Port 지원

## 참고사항

- PB0이 의도치 않게 켜질 수 있음 (이전 프로그램 설정 잔존)
- 물리적 리셋으로 해결 가능
- LED 밝기 조절은 소프트웨어 PWM으로 구현 가능

---

**작성일**: 2025-10-26  
**개발자**: STM32 테스트 프로젝트