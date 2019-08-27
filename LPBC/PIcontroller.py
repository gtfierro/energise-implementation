import numpy as np

class PIcontroller():
    def __init__(self, nphases, kp_ang, ki_ang, kp_mag, ki_mag):
        self.intError_ang = np.zeros(nphases)  # total integral error
        self.intError_mag = np.zeros(nphases)
        self.Pcmd_pu = np.zeros(nphases)
        self.Qcmd_pu = np.zeros(nphases)

        # controller gains must be a list, even if single phase. gains can be different on each phase
        # e.g. if only actuating on 2 phases (B and C) just put gains in order in list: [#gain B, #gain C]
        self.Kp_ang = kp_ang
        self.Ki_ang = ki_ang
        self.Kp_mag = kp_mag
        self.Ki_mag = ki_mag

    def PIiteration(self, nphases, phasor_error_mag, phasor_error_ang, sat_arrayP, sat_arrayQ):
        for phase in range(nphases):
            print('phase ' + str(phase))
            currentIntError_ang = (self.Ki_ang[phase] * phasor_error_ang[phase]) * sat_arrayP[phase]
            print('currentIntError_ang ' + str(currentIntError_ang))
            self.intError_ang[phase] += currentIntError_ang
            print('self.intError_ang[phase] ' + str(self.intError_ang[phase]))
            self.Pcmd_pu[phase] = (self.Kp_ang[phase] * phasor_error_ang[phase]) + self.intError_ang[phase]
            print('self.Pcmd_pu[phase] ' + str(self.Pcmd_pu[phase]))

            currentIntError_mag = (self.Ki_mag[phase] * phasor_error_mag[phase]) * sat_arrayQ[phase]
            self.intError_mag[phase] += currentIntError_mag
            self.Qcmd_pu[phase] = (self.Kp_mag[phase] * phasor_error_mag[phase]) + self.intError_mag[phase]
        return self.Pcmd_pu, self.Qcmd_pu  # positive for power injections
