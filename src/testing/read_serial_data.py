import serial
import operator
from functools import reduce
from timeit import default_timer
import matplotlib.pyplot as plt
import struct
import numpy as np
import pandas as pd
import seaborn as sns


fftsize=512

#def decodeX(data):
#    ints = list(map(int, d))
#    pairs = [ints[x:x+2] for x in range(0,len(ints),2)]
#    return np.array([(x[0]<<8)+x[1] for x in pairs])

def decode_int16_t(data, L):
    return struct.unpack('h'*L, data)


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
    fig, ax = plt.subplots(figsize=(8,9))
    #lpdf =  pd.DataFrame( np.hstack( ( np.expand_dims( np.array(range(ar.shape[0])), 1 ), ar ) ) ).melt(id_vars=(0,)).rename(columns={'variable':'bin', 0:'t'}).set_index(['bin'])
    #sns.lineplot( lpdf, x='bin', y='value', hue='t' )
    print(ar.shape)
    mean_spectrum =np.nanmean( ar, axis=0)

    print("Means:")
    print(mean_spectrum.min())
    print(np.median(mean_spectrum))
    print(mean_spectrum.max())
    peak = mean_spectrum[1:].argmax() + 1
    print(f"Peak: {peak}")
    plt.axvline(x=peak, ls='--', color='black')
    plt.annotate('f={:.3g}, y={:.3g}'.format(peak/ar.shape[1]*0.5, ar[:,peak].mean() ), xy=(peak, 1e4))
    plt.plot( mean_spectrum, label='mean' )

    #plt.plot( ar[:,100], label='100')
    #plt.plot( ar[:,300], label='300')
    ax.set_yscale('log')
    plt.xticks((0, fftsize//2, fftsize))
    ax.set_xticklabels(('0', '0.25f', '0.5f'))
    plt.grid(axis='both')
    plt.legend()
    plt.savefig('lineplot4.fHz.png', dpi=200)


    fig, ax = plt.subplots(figsize=(8,9))
    plt.imshow( np.log10(ar) )
    plt.xticks((0, fftsize//2, fftsize))
    ax.set_xticklabels(('0', '0.25f', '0.5f'))
    plt.colorbar()
    plt.savefig('waterfall4.fHz.png', dpi=200)




def readPackets(ser, L):
    buf = []
    bufLen = 0

    term = b'\x1f\xf0\x1f\xf0'

    while True:
        buf = bytes()
        d = ser.read_until(expected=term, size=L*2) #(512*2)

        buf = buf+d
        while term in buf:
            tpos = buf.index(term)
            if tpos>=L:
                ret = buf[:tpos]
                if len(ret)>L:
                    ret = ret[-L:]
                buf = buf[tpos+len(term):]
                yield ret
            else:
                buf = buf[tpos+len(term):]
                print(f"Warning: dropping1 {len(d)}")


with serial.Serial('/dev/ttyUSB0', 230400, timeout=1) as ser:
    #_ = ser.readline() # discard any partially received packet 
    #x = ser.readline() # use the second packet

    #print(x)
    #print(len(x))

    #analyze(x)

    ar = np.zeros(shape=(400,fftsize), dtype='int')
    print("Recording data...")
    t0 = default_timer()
    i = 0
    for d in readPackets(ser, L=fftsize*2):
        if(len(d) != fftsize*2):
            print(f"EEE got {len(d)}")
        assert(len(d)==fftsize*2)
        ar[i,:] = np.array( decode_int16_t(d, L=fftsize) )

        if i%100==99:
            t = default_timer()
            print(f"{t-t0}s N={i+1}")

        i += 1
        if i >= 400: break
    
    t = default_timer()
    print(f"(recorded 400 samples in {t-t0}s - {400/(t-t0)}/s)")

    print("Plotting data...")

    plot(ar)




