import numpy as np

class PIcontroller():
    def __init__(self, nphases, kp_ang, ki_ang, kp_mag, ki_mag):
        self.currentIntError_ang = np.zeros(nphases)
        self.currentIntError_mag = np.zeros(nphases)
        self.intError_ang = np.zeros(nphases)
        self.intError_mag = np.zeros(nphases)
        self.Pcmd_pu = np.zeros(nphases)
        self.Qcmd_pu = np.zeros(nphases)

        self.Kp_ang = kp_ang
        self.Ki_ang = ki_ang
        self.Kp_mag = kp_mag
        self.Ki_mag = ki_mag

    def PIiteration(self, nphases, phasor_error_mag, phasor_error_ang, sat_arrayP, sat_arrayQ):
        for phase in range(nphases):
            self.currentIntError_ang[phase] = (self.Ki_ang[phase] * phasor_error_ang[phase]) * sat_arrayP[phase]
            self.intError_ang[phase] += self.currentIntError_ang[phase]
            self.Pcmd_pu[phase] = (self.Kp_ang[phase] * phasor_error_ang[phase]) + self.intError_ang[phase]

            self.currentIntError_mag[phase] = (self.Ki_mag[phase] * phasor_error_mag[phase]) * sat_arrayQ[phase]
            self.intError_mag[phase] += self.currentIntError_mag[phase]
            self.Qcmd_pu[phase] = (self.Kp_mag[phase] * phasor_error_mag[phase]) + self.intError_mag[phase]
        return self.Pcmd_pu, self.Qcmd_pu