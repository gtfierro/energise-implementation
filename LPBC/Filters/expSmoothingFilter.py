
import numpy as np

class expSmoothingFilter():
    def __init__(self, filterAlpha, initialState = 0):
        #filter parameters:
        self.filterAlpha = filterAlpha #larger filter changes faster but will be noisier
        self.prev_state = initialState

    def filterStep(self,measurement): #expects measurement as a 1d array nphases-long
        self.prev_state = (1 - self.filterAlpha)*self.prev_state + self.filterAlpha*measurement
        return self.prev_state

'''
#Exponential Averaging Filter:

Can be thought of as a low pass filter. It will filter out the noise and high frequencies generally.
Downside is that it creates a signal delay

link explaining connection bn exponential smoothing and Kalman filter
https://gregstanleyandassociates.com/whitepapers/FaultDiagnosis/Filtering/Kalman-Filter/kalman-filter.htm
'''
