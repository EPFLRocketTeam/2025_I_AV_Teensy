import serial
import time

# Replace 'COM3' with your Arduino's serial port
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

# Give some time for the connection to initialize
time.sleep(2)

# Open a file to write the data
with open('sensor_data.csv', 'w') as f:
    while True:
        try:
            # Read a line of data from the serial port
            line = ser.readline().decode('utf-8').strip()
            if line:
                print(line)  # Optional: print to console
                f.write(line + '\n')
        except KeyboardInterrupt:
            print("Data logging stopped by user.")
            break

# Close the serial connection
ser.close()

