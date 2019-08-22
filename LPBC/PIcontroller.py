import numpy as np

class PIcontroller():
    def __init__(self, nphases, Kp_ang=0.01, Ki_ang=0.3, Kp_mag=0.01, Ki_mag=0.3):
        # INITIALIZATION
        self.intError_mag = np.zeros(nphases) #total integral error
        self.intError_ang = np.zeros(nphases)
        # K gains
        self.Kp_ang = Kp_ang
        self.Ki_ang = Ki_ang
        self.Kp_mag = Kp_mag
        self.Ki_mag = Ki_mag

    def PIiteration(self, Vmag_error, Vang_error, sat_arrayP, sat_arrayQ):
        #these are all ndarrays so multiplacation is element-wise, so works for scalars or vectors

        IntError_ang = Vang_error * sat_arrayP #saturation factored in here
        self.intError_ang += IntError_ang
        Pcmd = self.Kp_ang * Vang_error + self.Ki_ang * self.intError_ang

        IntError_mag = Vmag_error * sat_arrayQ
        self.intError_mag += IntError_mag
        Qcmd = self.Kp_mag * Vmag_error_pu + self.Ki_mag * self.intError_mag

        return(Pcmd, Qcmd) #these are postive for power injections, not extractions
