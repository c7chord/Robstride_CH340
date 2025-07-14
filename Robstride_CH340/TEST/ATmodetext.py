import serial
import time
import datetime

PORT = "/dev/ttyUSB0"
BAUDRATE = 921600

def realtime_logging():
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=0.5)
        print(f"✅ Connected to {PORT} at {BAUDRATE}bps")
        
        commands = [
            "41542b41540d0a", # AT+AT
            # "41549007e80c0805700000020000000d0a",
            "41541807e80c0800000000000000000d0a", # ENABLE
            "41549007e80c08187000000000b8410d0a", # SPEED TO 5
            "41549007e80c080a7000000000a0400d0a", # SPEED TO 5
        ]
        
        for i, cmd in enumerate(commands):
            timestamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
            print(f"\n[{timestamp}] --- Command {i+1} ---")
            
            # 전송
            data = bytes.fromhex(cmd)
            print(data)
            ser.write(data)
            print(f"[{timestamp}] 📤 Sent: {cmd}")
            
            # 응답 대기
            time.sleep(0.2)
            if ser.in_waiting > 0:
                response = ser.read(ser.in_waiting)
                timestamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
                print(f"[{timestamp}] 📥 Received: {response.hex()}")
                print(f"[{timestamp}] 📥 ASCII: {response}")
            else:
                timestamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
                print(f"[{timestamp}] 📥 No response")
            
            time.sleep(0.3)
        
        print("\n✅ All commands sent.")
        ser.close()
        
    except serial.SerialException as e:
        print(f"❌ Serial error: {e}")

if __name__ == "__main__":
    realtime_logging()