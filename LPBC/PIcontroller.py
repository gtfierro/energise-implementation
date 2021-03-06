import numpy as np
import copy

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
        phasor_error_ang = np.degrees(phasor_error_ang)
        for phase in range(nphases):
            # if controller is saturated and target/phasor error is same direction then turn on anti-windup,
            # this condition is necessary when kp << ki to get controller out of saturation
            if sat_arrayP[phase] == 0 and np.sign(self.intError_ang[phase]) == np.sign(phasor_error_ang[phase]):
                currentIntError_ang = phasor_error_ang[phase] * sat_arrayP[phase]
            else:  # if controller is saturated but target/phasor error is different direction then turn off anti-windup
                currentIntError_ang = phasor_error_ang[phase]
            self.intError_ang[phase] += currentIntError_ang

            self.Pcmd_pu[phase] = (self.Kp_ang[phase] * phasor_error_ang[phase]) + self.Ki_ang[phase] * self.intError_ang[phase]

            #SWITCHED Q to control angle for 33NF (low X/R ratio)
            #self.Qcmd_pu[phase] = (self.Kp_ang[phase] * phasor_error_ang[phase]) + self.Ki_ang[phase] * self.intError_ang[phase]

            if sat_arrayQ[phase] == 0 and np.sign(self.intError_mag[phase]) == np.sign(phasor_error_mag[phase]):
                currentIntError_mag = phasor_error_mag[phase] * sat_arrayQ[phase]
            else:
                currentIntError_mag = phasor_error_mag[phase]
            self.intError_mag[phase] += currentIntError_mag

            self.Qcmd_pu[phase] = (self.Kp_mag[phase] * phasor_error_mag[phase]) + self.Ki_mag[phase] * self.intError_mag[phase]

            #SWITCHED P to control magnitude for 33NF (low X/R ratio)
            #self.Pcmd_pu[phase] = (self.Kp_mag[phase] * phasor_error_mag[phase]) + self.Ki_mag[phase] * self.intError_mag[phase]

        print('self.intError_ang : ' + str(self.intError_ang))
        print('self.intError_mag : ' + str(self.intError_mag))
        return self.Pcmd_pu, self.Qcmd_pu  # positive for power injections
