import serial
#import matplotlib

def analyze(d):
    for i in range(0, len(d), 8):
        p = d[i:i+8]
        print(' '.join(map(str, map(ord, p))))


with serial.Serial('/dev/ttyUSB0', 115200, timeout=1) as ser:
    _ = ser.readline() # Dump the same
    x = ser.readline()
    print(x)
    print(len(x))

    analyze(x[:4096])



