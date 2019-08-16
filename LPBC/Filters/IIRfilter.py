
import numpy as np
from scipy import signal

class IIRbandstopFilter():
    def __init__(self, nphases, samplingFreq, stopbandFreqs, filterOrder, filterType='butter'): #stopband freqs in Hz (continuous domain)
        self.nphases= nphases
        self.samplingFreq = samplingFreq #this gives the discrete sampling period
        self.fn = np.asarray(samplingFreq)/2    # samplingPeriod in Hz
        self.stopband = stopbandFreqs / self.fn # dividing by fn_step puts the stopband frequencies in the discrete domain (making them unitless), compatible with iirfilter
        self.filterOrder = filterOrder
        self.filterType = filterType #default butterworth filter chosen so the passband is maximally flat and monotonic, rounded knee is fine for this application
        #build filter
        (self.b, self.a) = signal.iirfilter(filterOrder, self.stopband, btype='bandstop', analog=False, ftype=filterType)
        #filter initial state:
        self.zi = signal.lfilter_zi(self.b, self.a)
        self.z = np.outer(np.ones(nphases),self.zi) #this just puts a value in z, should call initializeFilterState before using filterStep

    def initializeFilterState(self,x0): #expects measurement as a 1d array nphases-long
        if self.nphases > 1:
            for i in range(self.nphases):
                self.z[i,:] = self.zi*x0[i]
        else:
            self.z = self.zi*x0

    def filterStep(self,measurement): #expects measurement as a 1d array nphases-long
        if self.nphases > 1:
            output = np.zeros(self.nphases)
            for i in range(self.nphases):
                (output[i],self.z[i,:]) = signal.lfilter(self.b, self.a, np.expand_dims(measurement[i],axis=0), zi=self.z[i,:])
        else:
            (output,self.z) = signal.lfilter(self.b, self.a, np.expand_dims(measurement,axis=0), zi=self.z)
        return output



'''
Delay Considerations:
The time delay introduced by the filter may create issues for the controller.
Higher filter order results in a larger delay of a new measurement making it out to the filtered signal.

different types of filters:
https://www.dsprelated.com/showarticle/164.php

internal state:
internal state is the state z such that the filter can be written as linear system (https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.lfilter_zi.html#scipy.signal.lfilter_zi)
Think this state is w in http://www.eas.uccs.edu/~mwickert/ece2610/lecture_notes/ece2610_chap8.pdf but havent confirmed This
storing w allows you to jsut store one value, rather than previous x's and y's

For going b/n analog and digital w sampling:
an analog frequency fc gives a digital frequency f = T*f and a digital radian frequency w = 2*np.pi*T*fc
T is 1/fs, where fs is the sampling frequency
fc is how many seconds the signal takes to repeat (units 1/s), w is how many samples the signal takes to repeat (unitless)
Digital frequencies offset by 2pi are indistinguishable, so frequencies +/- 2pi*k higher (or lower) will be mapped onto the digital frequency of interest
This happens for discrete and not continuous signals bc steps [n] are always integers, while time (t) is a continuum
https://allsignalprocessing.com/discrete-time-frequency-avoid-confusion/

frequency f is in Hertz = cycles per seconds
radian frequency w is in radians/second

In PMU applications we never see an analog signal, just a digital signal that we downsample (decimate)
if (down)sampling frequency is 1/4 Hz, then the nyquist frequency is 1/8 Hz

Aliasing:
Aliasing is only an issue if you are using the frequency spectrum.
For our application (which is time domain), it doesn't matter.

Decimation:
https://dspguru.com/dsp/faqs/multirate/decimation/
"Specifically, the signal’s highest frequency must be less than half the post-decimation sampling rate.""
Need to LPF the digital signal so that no signals above fn = 1/(steprate*2) are present, then downsample
This is only necessary if you want to preserve the frequency spectrum characteristics
But since we are using a time domain controller, we dont really need to preserve the time domain characteristics
Even if we are bandstopping a given band, that will just stop the higher order harmonics as well

Question is, if we put in a filter before the downsampling, what do we set as the LPF frequency?
Doesnt have to be at the sampling nyquist frequency 1/(steprate*2) becuase we dont really care about aliasing
(we could just do exponenetial smoothing, which is a time domain LPF)

Alternative to lfilt:
# manually apply filter:
# prevMeas = data2[0:len(a)-1]
# for i in range(N):
#     if i >= len(a):
#         #uses the len(b) data2 points up to (i-1), and len(a)-1 data2f points up to data2f[i-1]
#         data2f[i] = np.dot(b,data2[i-len(b):i]) - np.dot(a[1:],prevMeas)
#         prevMeas[1:] = prevMeas[0:-1]
#         prevMeas[0] = data2f[i]


'''
