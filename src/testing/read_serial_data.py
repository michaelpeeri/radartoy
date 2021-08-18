import serial
from timeit import default_timer
import matplotlib.pyplot as plt
import numpy as np


def decode(d):
    ints = list(map(int, d))
    pairs = [ints[x:x+2] for x in range(0,len(ints),2)]
    return np.array([(x[0]<<8)+x[1] for x in pairs])


def analyze(d):
    for i in range(0, len(d), 8):
        p = d[i:i+8]
        print(' '.join(map(str, map(int, p))))
    
    #nd0 = np.array([n for i, n in enumerate(map(int, d)) if i%2==1])
    #nd1 = np.array([n for i, n in enumerate(map(int, d)) if i%2==0])

    W=64

    ints = list(map(int, d))
    print(len(ints))
    ints = ints[:len(ints)//W*W]
    print(len(ints))
    pairs = [ints[x:x+2] for x in range(0,len(ints),2)]
    nd1 = np.array([(x[0]<<8)+x[1] for x in pairs])
    nd2 = np.array([(x[1]<<8)+x[0] for x in pairs])


    fig, ax = plt.subplots(figsize=(12,6))
    plt.plot(range(0,512),   np.log10(nd1), c='blue',  label='fft1', alpha=0.6)
    #plt.plot(range(0,256),   np.log10(nd1[0:512:2]), c='blue',  label='fft1', alpha=0.6)
    #plt.plot(range(256,512), np.log10(nd1[1:512:2]), c='blue',  label='fft1', alpha=0.6)
    plt.plot(range(0,512),   np.log10(nd2), c='red',   label='fft2', alpha=0.6)
    #plt.plot(range(0,256),   np.log10(nd2[0:512:2]), c='red',   label='fft2', alpha=0.6)
    #plt.plot(range(256,512), np.log10(nd2[1:512:2]), c='red',   label='fft2', alpha=0.6)
    plt.axvline(x=256, c='black')
    plt.axvline(x=193, c='black', ls='--', zorder=0, alpha=0.5)
    plt.xticks((0,128,256,384,512))
    ax.set_xticklabels(('0', '0.125f', '0.25f', '0.375f','0.5f'))
    plt.savefig('test0.png', dpi=200)

    fig, ax = plt.subplots(figsize=(12,9))
    plt.imshow( np.reshape(nd1, (W,-1)).T )
    plt.savefig('test1.png', dpi=200)

    fig, ax = plt.subplots(figsize=(12,9))
    plt.imshow( np.reshape(nd2, (W,-1)).T )
    plt.savefig('test2.png', dpi=200)

    #plt.ylim((0,255))
    #plt.legend()

def plot(ar):
    fig, ax = plt.subplots(figsize=(12,9))
    plt.imshow( ar )
    plt.xticks((0,128,256,384,512))
    ax.set_xticklabels(('0', '0.125f', '0.25f', '0.375f','0.5f'))
    plt.colorbar()
    plt.savefig('test_ar.png', dpi=200)


with serial.Serial('/dev/ttyUSB0', 921600, timeout=1) as ser:
    #_ = ser.readline() # discard any partially received packet 
    #x = ser.readline() # use the second packet

    #print(x)
    #print(len(x))

    #analyze(x)


    ar = np.zeros(shape=(400,512), dtype='int')
    print("Recording data...")
    t0 = default_timer()
    for i in range(400):
        d = ser.read(512*2)
        if i%100==99:
            t = default_timer()
            print(f"{t-t0}s N={i+1}")
        ar[i,:] = decode(d)
    
    t = default_timer()
    print(f"(recorded 400 samples in {t-t0}s - {400/(t-t0)}/s)")

    print("Plotting data...")

    plot(ar)




