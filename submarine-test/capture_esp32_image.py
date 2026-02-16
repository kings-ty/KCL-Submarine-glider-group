import serial
import time
import os
from PIL import Image

# --- 설정 부분 ---
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
OUTPUT_FILENAME = 'captured_image.jpg'
# --- 설정 끝 ---

def capture_image_from_esp32():
    ser = None
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=5)
        ser.rts = False
        ser.dtr = False
        
        print(f"Connected to serial port {SERIAL_PORT} at {BAUD_RATE} baud.")
        ser.flushInput()
        time.sleep(1)

        print("Waiting for ESP32-CAM to be ready...")
        ready_line = b''
        start_time = time.time()
        while b'Camera initialized' not in ready_line:
            if ser.in_waiting > 0:
                ready_line += ser.read(ser.in_waiting)
            else:
                time.sleep(0.01)
            if time.time() - start_time > 15:
                print("Timeout waiting for ESP32-CAM ready signal.")
                print(f"Received so far: {ready_line.decode(errors='ignore')}")
                ser.close()
                return
        
        print("ESP32-CAM is ready.")
        ser.flushInput()

        print("Sending capture command 'c'...")
        ser.write(b'c') 
        
        # ======================================================================
        # LOGIC FIX: Read lines until we find the "SIZE:" line
        # ======================================================================
        size_line = b''
        while True:
            line = ser.readline()
            if not line: # Timeout
                print("Timeout waiting for SIZE: line from ESP32-CAM.")
                ser.close()
                return
            
            print(f"ESP32-CAM says: {line.decode(errors='ignore').strip()}")
            if b'SIZE:' in line:
                size_line = line
                break # Found the size line, exit the loop
        # ======================================================================

        try:
            size_str = size_line.split(b'SIZE:')[1].strip()
            image_size = int(size_str)
            print(f"Image size reported: {image_size} bytes.")
        except (ValueError, IndexError):
            print(f"Error parsing image size from: {size_line.decode(errors='ignore')}")
            ser.close()
            return

        print("Receiving image data...")
        ser.timeout = 10
        image_data = ser.read(image_size)
        bytes_received = len(image_data)
        
        if bytes_received >= image_size:
            print(f"Successfully received {bytes_received} bytes of image data.")
            
            with open(OUTPUT_FILENAME, 'wb') as f:
                f.write(image_data)
            print(f"Image saved as {OUTPUT_FILENAME}")

            try:
                img = Image.open(OUTPUT_FILENAME)
                img.show()
                print("Image displayed.")
            except Exception as e:
                print(f"Could not display image: {e}")

        else:
            print(f"Incomplete image data received. Expected {image_size}, got {bytes_received}.")

    except serial.SerialException as e:
        print(f"Serial port error: {e}")
        print("Please ensure the Arduino Serial Monitor is closed and the correct port is selected.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        if ser and ser.is_open:
            ser.close()
            print("Serial port closed.")

if __name__ == "__main__":
    try:
        from PIL import Image
    except ImportError:
        print("Pillow library not found. Please install it using: pip install Pillow")
        exit()
        
    capture_image_from_esp32()