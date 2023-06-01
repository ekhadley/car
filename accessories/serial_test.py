import numpy as np
import serial, time

hold = 1
while hold:
    try:
        qaz = serial.Serial('COM6', 9600, timeout=.1)
        hold = 0
    except Exception:
        time.sleep(1)
        print('CONNECT FAILED')

i = 0
while 1:
    ctrl = (input("Okayeg WAT SEDN? : ").encode())
    #ctrl = str(i).encode()

    while qaz.in_waiting > 0:
        print(qaz.readline().decode())
 
    qaz.write(ctrl)





"""this is a comment xDDDDDDDDDDDDDDDDDD"""






























