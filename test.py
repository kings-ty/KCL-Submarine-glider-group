import serial
import time

print("=== 정상 모드 부팅 시퀀스 ===\n")
print("1. USB를 뽑으세요")
input("   뽑았으면 Enter...")

print("\n2. 5초 대기 중...")
time.sleep(5)

print("\n3. 이제 USB를 꽂으세요 (버튼 누르지 마세요!)")
input("   꽂았으면 Enter...")

print("\n4. 3초간 버튼을 누르지 마세요!")
for i in range(3, 0, -1):
    print(f"   {i}초...", end='\r')
    time.sleep(1)

print("\n\n5. 테스트 중...")
time.sleep(1)

try:
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    ser.reset_input_buffer()
    
    # AT 명령
    ser.write(b'AT\r\n')
    time.sleep(0.5)
    
    response = ser.read(ser.in_waiting)
    print(f"\n응답: {response}")
    
    if not response:
        print("\n⚠️  여전히 응답 없음")
        print("투명 전송 모드이거나 다른 Baud Rate일 수 있습니다")
    elif b'?>' in response:
        print("\n❌ 여전히 펌웨어 모드")
        print("버튼을 누르지 않고 다시 시도하세요")
    else:
        print("\n✅ 정상 응답!")
    
    ser.close()
    
except Exception as e:
    print(f"\n❌ 오류: {e}")