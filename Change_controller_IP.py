import serial
import time

COM_PORT = "COM7"        # prolific port number
BAUDRATE = 115200
NEW_IP = "192,168,0,123"

def change_ip():
    with serial.Serial(
        port=COM_PORT,
        baudrate=BAUDRATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=1
    ) as ser:

        time.sleep(1)

        cmd = f"IP{NEW_IP}\r"
        ser.write(cmd.encode("ascii"))
        time.sleep(0.2)

        ser.write(b"IP\r")
        time.sleep(0.2)

        response = ser.read_all().decode(errors="ignore")
        print("Controller response:")
        print(response)

if __name__ == "__main__":
    change_ip()
