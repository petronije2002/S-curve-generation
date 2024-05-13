import serial
import matplotlib.pyplot as plt

# Define the serial port and baud rate
serial_port = '/dev/tty.usbserial-110'  # Replace 'COMx' with your Arduino's serial port
baud_rate = 115200

# Open the serial port
ser = serial.Serial(serial_port, baud_rate)

# Lists to store data
time_list = []
velocity_list = []
max_velocity_list = []

# Read and parse data from Arduino
while True:
    try:
        # Read a line of data from serial
        data = ser.readline().decode().strip()
       
            # Split the data into components
          
        if data:
            # print(data)  # Print the data received from Arduino
            # Split and convert data to floats
            print(data)
            t, max_v, v_cmd = map(float, data.split(', '))

            # Append data to lists
            time_list.append(t)
            velocity_list.append(v_cmd)
            max_velocity_list.append(max_v)

            # Print the data (optional)
            # print(f"Time: {t}, Max Velocity: {max_v}, Command Velocity: {v_cmd}")
    except KeyboardInterrupt:
        # Stop reading data if KeyboardInterrupt (Ctrl+C) is detected
        break

# Close the serial port
ser.close()

# Plot the data
plt.figure(figsize=(10, 6))
plt.plot(time_list, velocity_list, label='Command Velocity')
plt.plot(time_list, max_velocity_list, label='Max Velocity', linestyle='--')
plt.xlabel('Time')
plt.ylabel('Velocity')
plt.title('Velocity vs Time')
plt.legend()
plt.grid(True)
plt.show()
