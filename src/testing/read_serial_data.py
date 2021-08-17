import serial
import matplotlib.pyplot as plt
import numpy as np

def analyze(d):
    for i in range(0, len(d), 8):
        p = d[i:i+8]
        print(' '.join(map(str, map(int, p))))
    
    nd0 = np.array([n for i, n in enumerate(map(int, d)) if i%2==1])
    nd1 = np.array([n for i, n in enumerate(map(int, d)) if i%2==0])
    fig, ax = plt.subplots()
    plt.plot(nd0, c='blue', label='ADC1')
    plt.plot(nd1, c='red',  label='ADC2')
    plt.ylim((0,255))
    plt.legend()
    plt.savefig('test.png')



with serial.Serial('/dev/ttyUSB0', 230400, timeout=1) as ser:
    #_ = ser.readline() # discard any partially received packet 
    #x = ser.readline() # use the second packet
    x = ser.read(1024)
    #print(x)
    print(len(x))

    analyze(x)



