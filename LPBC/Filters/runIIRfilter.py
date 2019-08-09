

from IIRfilter import *

import numpy as np
import matplotlib.pyplot as plt

import sys

# parameters
nphases = 1
stepRate = 4 #this gives the discrete sampling rate
filterOrder = 3
stopbandFreqs = (1/100, 1/15) # in cont domain , Hz
decimation = 1 #1 if fs_step is used (filtering after decimation), 0 if fs_max is used (filtering before decimation)

# create time data
tmax = 400   # sec
fs_max = 120            # Hz (max sampling rate for PMUs is 120 Hz)
fs_step = 1/stepRate    # Hz (sampling frequency at which step is called)
if decimation == 1:
    time = np.arange(start=0, step=1/fs_step, stop=tmax) #this is an implicit decimation
else:
    time = np.arange(start=0, step=1/fs_max, stop=tmax)

# define artificial data
N = time.size
data0 = np.ones(N).astype(np.float)         # data0 - signal without noise (constant value)
data1 = data0  #+ 0.002 * np.random.randn(N)    # data1 - signal with wideband noise (if # is removed)
fdisturbance = 1/30                           # narrowband disturbance frequency in Hz
w = 2 * np.pi * fdisturbance                # angular frequency (still continuous domain)
data2 = data1 + 0.02 * np.cos(w * time)      # data2 - signal with wideband and narrowband noise, sampling at fs_max or fs_step is implict here in the usage of the time vector
#to test multiple dimension data: (also need ot set nphases to 2)
#data2 = np.asarray((data2, data2))

#build filter
filter = IIRbandstopFilter(nphases, 1/stepRate, stopbandFreqs, filterOrder, filterType='butter')
#for non-downsampled data:
# filter = IIRBandstopFilter(nphases, fs_max, stopbandFreqs, filterOrder, filterType='butter')

# apply the filter to artificial data in real time
if nphases > 1:
    #initialize:
    data2_filtered = np.zeros((nphases,N))
    filter.initializeFilterState(data2[:,0])
    #run
    for i in range(N):
        data2_filtered[:,i] = filter.filterStep(data2[:,i])
    #plot
    plotindex = 0
    plt.plot(time, data2_filtered[plotindex,:], label='filtered')
    plt.plot(time, data2[plotindex,:], label='unfiltered')
    plt.plot(time, data0, label='target')
    plt.title('Vang filtered')
    plt.ylabel('Vang')
    plt.xlabel('Timestep')
    plt.legend()
    plt.show()
else:
    #initialize:
    data2_filtered = np.zeros(N)
    filter.initializeFilterState(data2[0])
    #run
    for i in range(N):
        data2_filtered[i] = filter.filterStep(data2[i])
    #plot
    plt.plot(time, data2_filtered, label='filtered')
    plt.plot(time, data2, label='unfiltered')
    plt.plot(time, data0, label='target')
    plt.title('Vang filtered')
    plt.ylabel('Vang')
    plt.xlabel('Timestep')
    plt.legend()
    plt.show()
