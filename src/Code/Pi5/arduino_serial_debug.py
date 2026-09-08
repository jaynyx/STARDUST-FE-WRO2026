import serial


arduino = serial.Serial("/dev/ttyACM1", 115200)

while True:
    command = input()
    arduino.write(command.encode())
