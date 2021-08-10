import serial
import matplotlib.pyplot as plt
import numpy as np

def analyze(d):
    for i in range(0, len(d), 8):
        p = d[i:i+8]
        print(' '.join(map(str, map(int, p))))
    
    nd = np.array([n for i, n in enumerate(map(int, d)) if i%2==1])
    fig, ax = plt.subplots()
    plt.plot(nd)
    plt.savefig('test.png')



with serial.Serial('/dev/ttyUSB0', 115200, timeout=1) as ser:
    _ = ser.readline() # Dump the same
    x = ser.readline()
    print(x)
    print(len(x))

    analyze(x[:4096])



