import serial
import getch
serialport = serial.Serial ("/dev/ttyS0")
serialport.baudrate = 115200
while(True):
    x = getch.getch().lower()
    # type your code here
    if x == "p":
        break
    if x == "w":
        command = "+100+10015+00"
    if x == "s":
        command = "-100-10015+00"
    if x == "a":
        command = "+050+10015+00"
    if x == "d":
        command = "+100+05015+00"
    if x == "m":
        command = "+000+00015+00"
    serialport.write(command.encode())