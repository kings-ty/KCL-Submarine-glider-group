import serial
import time

# --- 设置 (根据您的环境进行修改) ---
# Linux (Ubuntu/RPi) 通常为 '/dev/ttyACM0' 或 '/dev/ttyUSB0'
# Windows 为 'COM3' 之类的格式。
PORT = '/dev/ttyUSB0' 
BAUD_RATE = 115200
# ------------------------------------

def run_serial_monitor():
    try:
        # 打开串口
        ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
        print(f"--- 串口监视器已启动 [{PORT}] ---")
        time.sleep(2) # 等待连接稳定
        
        # 清空 Arduino 缓冲区 (防止启动时的乱码)
        ser.flushInput()

        while True:
            if ser.in_waiting > 0:
                # 1. 读取一行
                line = ser.readline().decode('utf-8').rstrip()
                
                # 2. 与当前时间一起输出 (比 Arduino IDE 好多了吧？)
                current_time = time.strftime("[%H:%M:%S]")
                print(f"{current_time} {line}")
                
                # (可选) 如果想保存到特定的文件，请取消下面的注释
                # with open("sensor_log.txt", "a") as f:
                #     f.write(f"{current_time} {line}\n")

    except serial.SerialException as e:
        print(f"\n[Error] 无法打开端口: {e}")
        print("请检查端口号是否正确，以及 Arduino IDE 的串口监视器是否已打开！")
    except KeyboardInterrupt:
        print("\n--- 用户已中断。正在关闭监视器。 ---")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    run_serial_monitor()

# Last modified: 2026-02-15T14:04:49Z