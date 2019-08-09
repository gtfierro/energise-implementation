

from pyxbos.process import run_loop, config_from_file #https://github.com/gtfierro/xboswave/blob/master/python/pyxbos/pyxbos/process.py
from pyxbos.drivers import pbc  #https://github.com/gtfierro/xboswave/tree/master/python/pyxbos/pyxbos/drivers
# above imports LPBCProcess, SPBCProcess, EnergiseMessage, LPBCStatus, LPBCCommand, SPBC, EnergiseError
import sys
import numpy as np
import warnings
import logging
import requests
from requests_futures.sessions import FuturesSession

warnings.simplefilter(action='ignore', category=FutureWarning)
logging.basicConfig(level="INFO", format='%(asctime)s - %(name)s - %(message)s')

from PIcontroller import *


#to use session.get for parallel API commands you have to download futures: pip install --user requests-futures

'''
next steps
auto-set toml files

APC impedance estimation from commands
'''

'''
from: https://github.com/gtfierro/energise-implementation/blob/master/LPBC/lpbc-example.py
Configuration:
- `namespace`: do not change
- `wavemq`: address of local wavemq agent
- `name`: name of the LPBC controller. **This needs to be unique**
- `entity`: the name of the local file constituting the 'identity' of this process.
  The entity file is what gives this process the permission to interact with other
  resources. File is created by `create_lpbc.sh`
- `spbc`: the name of the SPBC this LPBC is subscribed to for phasor targets
- `local_channels`: a list of URIs representing the phasor channels the LPBC
  subscribes to as the local measurement phasors
- `reference_channels`: a list of URIs representing the phasor channels the LPBC
  subscribes to as reference phasors
- `rate`: how many seconds between executions of the LPBC (can be fractional, e.g. 0.5)
'''
class lpbcwrapper(pbc.LPBCProcess): #this is related to super(), inherits attributes and behaviors from pbc.LPBCProcess (which is a wrapper for XBOSProcess)
    def __init__(self, cfg, nphases, act_idxs, actType, PMUchannels_to_phase_Map, currentMeasExists, localSratio=1, localVratio=1):
        super().__init__(cfg)

        # INITIALIZATION
        self.controller = PIcontroller(nphases)
        self.ametek_phase_shift = 0

        self.nphases = nphases
        self.iteration_counter = 0

        self.Pcmd_kVA = np.zeros(nphases)
        self.Qcmd_kVA = np.zeros(nphases) #Pcmd comes from the feedback controller
        self.Pcmd_pu = np.zeros(nphases)  #both Pcmd and Pact are in the local power setting (not the amplified OpalRT setting which is just multiplied by localSratio)
        self.Qcmd_pu = np.zeros(nphases)

        self.Pact_kVA = np.zeros(nphases) #Pactual (measured from pmus, used to calc saturation)
        self.Qact_kVA = np.zeros(nphases)
        self.Pact_pu = np.zeros(nphases)
        self.Qact_pu = np.zeros(nphases)

        self.Vang = 'initialize'
        self.Vmag = np.zeros(nphases)
        self.Vmag_pu = np.zeros(nphases)
        self.Vmag_relative = np.zeros(nphases)
        self.Vmag_relative_pu = np.zeros(nphases)
        self.phasor_error_ang = np.zeros(nphases)
        self.phasor_error_mag_pu = np.zeros(nphases)
        self.VmagRef = np.zeros(nphases)
        self.VmagRef_pu = np.zeros(nphases)

        #Just need to decide what to call unintialized values (probably np.zero if more than 1 dimension)
        #Targets received from SPBC
        self.VangTarg = 'initialize'    #intialized the first time a phasor_target packet comes from the SPBC, control loop isnt run until a packet is received
        self.VmagTarg = 'initialize'
        self.VmagTarg_pu = np.zeros(nphases)
        self.VmagTarg_relative = np.zeros(nphases)
        self.VmagTarg_relative_pu = np.zeros(nphases)

        self.kVbase = np.NaN #received from SPBC #intialized the first time a phasor_target packet comes from the SPBC, control loop isnt run until a packet is received
        self.localVratio = localVratio #!= 1 if Ametek voltage ratio needs to be taken into account (ie PMU123 not PMUP123 used for the voltage)
        self.localkVbase = np.NaN # = self.kVbase/self.localVratio
        self.network_kVAbase = np.NaN #received from SPBC
        self.localSratio = localSratio #ratio between actual power and power in Opal siulation, eg 500/3.3
        self.localkVAbase = np.NaN # = self.network_kVAbase/self.localSratio
        self.localIbase = np.NaN # = self.network_kVAbase/self.localkVbase

        self.PMUplugs_to_phase_map = PMUplugs_to_phase_map #3-entry vector that maps PMU channels to the true phases (SPBC commands should be given in terms of the true phases)
        self.PMUplugs_to_phase_idx = PMUplugs_to_phase_map[PMUplugs_to_phase_map != ''] #(not used really) PMUplugs_to_phase_map with the empty strings removed
        self.PMUplugs_to_V_idx = [0] * nphases #maps the plugs (ordered L1 L2 L3) of the PMU to the entries of the V (or I) vectors such that the order is always A then B then C (when each phase is available available)
        #if 2 or 3 phase do below: (if its single phase you can just leave it)
        if nphases > 1:
            if 'A' in self.PMUplugs_to_phase_idx[:nphases]: #takes care of the 3-phase, and 2 of the 3 2-phase scenarios # dont the :nphases if PMUplugs_to_phase_map is 3-long
                for i, phase in enumerate(self.PMUplugs_to_phase_idx[:nphases]): #[:nphases] gives the first nphases values (so it takes care of the cases when current measurements are included as well)
                    if phase == 'A':
                        self.PMUplugs_to_V_idx[i] = 0 #eg if L2 maps to A then then second entry of self.PMUplugs_to_V_idx will be 0
                    if phase == 'B':
                        self.PMUplugs_to_V_idx[i] = 1
                    if phase == 'C':
                        self.PMUplugs_to_V_idx[i] = 2 - (3 - nphases) #writes a 1 if just A and C
            else:
                for i, phase in enumerate(self.PMUplugs_to_phase_idx[:nphases]): #takes care of the case when just B and C phases are present
                    if phase == 'B':
                        self.PMUplugs_to_V_idx[i] = 0
                    if phase == 'C':
                        self.PMUplugs_to_V_idx[i] = 1
        #Maps PMU data to the voltage vact in the order A, B, C (according to availability) using PMUplugs_to_phase_map

        #current measurements
        self.Iang = 'initialize'
        self.Imag = np.zeros(nphases)
        self.Imag_pu = np.zeros(nphases)
        self.Icomp_est = np.zeros(nphases,dtype=np.complex_)
        self.Icomp_pu_est = np.zeros(nphases,dtype=np.complex_)

        #saturation variables
        self.saturated = 0 #1 if all actuators are saturated
        self.sat_arrayP = np.ones(nphases) #logic vectors which are 1 if a given phase is not saturated, and zero if it is
        self.sat_arrayQ = np.ones(nphases) #if no current measurements, then these will just stay zero and saturated == 0
        self.Pmax_pu = np.asarray([np.NaN] * nphases) #this signal is used by the SPBC if ICDI is true, otherwise its a nan
        self.Qmax_pu = np.asarray([np.NaN] * nphases)
        self.saturationCounterLimit = 5
        self.Psat = np.ones((nphases, self.saturationCounterLimit))
        self.Qsat = np.ones((nphases, self.saturationCounterLimit))
        self.ICDI_sigP = np.zeros((nphases, 1), dtype=bool) #I Cant Do It signal
        self.ICDI_sigQ = np.zeros((nphases, 1), dtype=bool)

        #phasor calc
        self.local_time_index = [np.NaN]*nphases
        self.ref_time_index = [np.NaN]*nphases
        self.nPhasorReadings = 50
        self.pmuTimeWindow = 2000000 #in ns, 2000000 is 2 ms

        # https config
        #these are the actuators (inverters) that are controlled by a given lpbc. inverters are counted off 1,2,3, loads are counted off 0,1,2
        self.act_idxs = np.asarray(act_idxs)
        self.actuatorType = actType #'inverter' or 'load'
        if self.type == 'inverter':
            self.act_idxs = self.act_idxs + 1 #inverters indexed starting with 1 not 0
        commandReceipt = np.zeros(nphases)

        #Flexlab specific commands
        self.currentMeasExists = currentMeasExists
        self.loadrackPlimit = 2000. #size of a load rack in VA
        self.batt_max = 3300.
        self.inv_s_max = 7600.
        self.mode = 1  #Howe we control inverters mode 1: PV as disturbance, mode 2: PV calculated, mode 3: PV only
        self.batt_cmd = np.zeros((nphases, 1)) #battery commands are given in watts
        self.invPperc_ctrl = np.zeros((nphases, 1)) #inverter P commnads are given as a percentage of inv_s_max
        self.load_cmd = np.zeros((nphases, 1)) #load commands are given in watts


    def targetExtraction(self,phasor_target):
        #this implies A,B,C order to measurements from SPBC
        Vmag_targ_dict = dict()
        Vang_targ_dict = dict()
        kvbase_dict = dict()
        kvabase_dict = dict()
        phaseA = False
        phaseB = False
        phaseC = False
        for i in np.arrange(nphases):
            if 'ph_A' in phasor_target['phasor_targets'][i]['channelName']:
                phaseA = True
                phase = 'A'
            elif 'ph_B' in phasor_target['phasor_targets'][i]['channelName']:
                phaseB = True
                phase = 'B'
            elif 'ph_C' in phasor_target['phasor_targets'][i]['channelName']:
                phaseC = True
                phase = 'C'
            else:
                disp('no phase found for target ' + str(i) + 'using phase A')
                phaseA = True
                phase = 'A'
            Vmag_targ_dict[phase] = phasor_target['phasor_targets'][i]['magnitude']
            Vang_targ_dict[phase] = phasor_target['phasor_targets'][i]['angle']
            kvbase_dict[phase] = phasor_target['phasor_targets'][i]['kvbase']['value']
            kvabase_dict[phase] = phasor_target['phasor_targets'][i]['kvabase']['value']
        Vmag_targ = []
        Vang_targ = []
        kvbase = []
        kvabase = []
        if phaseA:
            Vmag_targ.append(Vmag_targ_dict['A'])
            Vang_targ.append(Vang_targ_dict['A'])
            kvbase.append(kvbase_dict['A'])
            kvabase.append(kvabase_dict['A'])
        if phaseB:
            Vmag_targ.append(Vmag_targ_dict['B'])
            Vang_targ.append(Vang_targ_dict['B'])
            kvbase.append(kvbase_dict['B'])
            kvabase.append(kvabase_dict['B'])
        if phaseC:
            Vmag_targ.append(Vmag_targ_dict['C'])
            Vang_targ.append(Vang_targ_dict['C'])
            kvbase.append(kvbase_dict['C'])
            kvabase.append(kvabase_dict['C'])
        return (Vmag_targ, Vang_targ, kvbase, kvabase)
        #there are alternative ways to do this (eg creating Phases_to_V_idx using similar logic)



    #also need this to work for current, or write a different one
    def phasorV_calc(self, local_phasors, reference_phasors, nphases, PMUplugs_to_V_idx):
        # Initialize
        local = [0] * nphases # makes a list nphases-long, similar to np.zeros(nphases), but a list
        ref = [0] * nphases
        flag = [1] * nphases #used to check if a phasor match was found
        # Extract latest nPhasorReadings readings from local and ref uPMUs
        for plug in range(nphases): #this will just read the voltage measurements cause its nphases long, even if local_phasors also has current measurements
            if len(local_phasors[plug]) > self.nPhasorReadings:
                dataWindowLength = self.nPhasorReadings
            else:
                dataWindowLength = len(local_phasors[plug])
            local[plug] = local_phasors[plug][-dataWindowLength:] #from dataWindowLength back to present, puts Lx2 entries in each entry of local, x2 is for magnitude and phase
            ref[plug] = reference_phasors[plug][-dataWindowLength:]
        if self.Vang == 'initialize':
            self.Vang = np.zeros(nphases)
            # loops through every phase with actuation
            for plug in range(nphases):
                # Initialize: Extract measurements from most recent timestamps only for first iteration
                #this isnt the most robust way to do this
                phase_idx = PMUplugs_to_V_idx[plug]
                V_mag_local = local[plug][-1]['magnitude']
                V_ang_local = local[plug][-1]['angle'] - self.ametek_phase_shift
                V_mag_ref = ref[plug][-1]['magnitude']
                V_ang_ref = ref[plug][-1]['angle']
                self.Vang[phase_idx] = V_ang_local - V_ang_ref
                self.Vmag[phase_idx] = V_mag_local
                self.VmagRef[phase_idx] = V_mag_ref
                self.Vmag_relative[phase_idx] = V_mag_local - V_mag_ref
        # loops through each set of voltage measurements for each phase
        local_time_index = [np.NaN]*nphases
        ref_time_index = [np.NaN]*nphases
        for plug in range(nphases): #plug is the number of the PMU plug (0, 1 or 2)
            phase_idx = PMUplugs_to_V_idx[plug] #this orders the readings from the plug into A,B,C order in self.Vang, self.Vmag etc.
            # loops through every local uPMU reading starting from most recent
            for local_packet in reversed(local[plug]):
                # extract most recent local uPMU reading
                local_time = int(local_packet['time'])
                # loops though every reference uPMU reading starting from most recent
                for ref_packet in reversed(ref[plug]):
                    ref_time = int(ref_packet['time'])
                    # check timestamps of local and reference uPMU if within 2 ms
                    if abs(ref_time - local_time) <= self.pmuTimeWindow:
                        local_time_index[phase_idx] = local[plug].index(local_packet)
                        ref_time_index[phase_idx] = ref[plug].index(ref_packet)
                        # Extract measurements from closest timestamps
                        V_mag_local = local[plug][local_time_index[phase_idx]]['magnitude']
                        V_ang_local = local[plug][local_time_index[phase_idx]]['angle'] - self.ametek_phase_shift
                        V_mag_ref = ref[plug][ref_time_index[phase_idx]]['magnitude']
                        V_ang_ref = ref[plug][ref_time_index[phase_idx]]['angle']
                        # calculates relative phasors
                        self.Vang[phase_idx] = V_ang_local - V_ang_ref
                        self.Vmag[phase_idx] = V_mag_local
                        self.VmagRef[phase_idx] = V_mag_ref
                        self.Vmag_relative[phase_idx] = V_mag_local - V_mag_ref
                        flag[plug] = 0
                        break
                if flag[plug] == 0:
                    break
            if flag[plug] == 1:
                print("PMU plug", plug, ", Iteration ", self.iteration_counter, ": No timestamp found")
        return (self.Vang,self.Vmag,self.VmagRef,self.Vmag_relative, local_time_index, ref_time_index, dataWindowLength) #returns the self. variables bc in case a match isnt found, they're already initialized



    def phasorI_calc(self, local_time_index, ref_time_index, dataWindowLength, local_phasors, reference_phasors, nphases, PMUplugs_to_V_idx):
        #uses the same time indeces that were found from the voltage search
        # Initialize
        local = [0] * nphases # makes a list nphases-long, similar to np.zeros(nphases), but a list
        ref = [0] * nphases
        for plug in range(nphases):
            local[plug] = local_phasors[plug + nphases][-dataWindowLength:] #from dataWindowLength back to present, puts Lx2 entries in each entry of local, x2 is for magnitude and phase
            ref[plug] = reference_phasors[plug + nphases][-dataWindowLength:] #plug + nphases selects the current data rather than the voltage data
        if self.Iang == 'initialize':
            self.Iang = np.zeros(nphases)
            for plug in range(nphases):
                # Initialize: Extract measurements from most recent timestamps only for first iteration
                #this isnt the most robust way to do this
                phase_idx = PMUplugs_to_V_idx[plug]
                self.Imag[phase_idx] = local[plug][-1]['magnitude']
                I_ang_local = local[plug][-1]['angle']
                I_ang_ref = ref[plug][-1]['angle']
                self.Iang[phase_idx] = I_ang_local - I_ang_ref
        for plug in range(nphases):
            phase_idx = PMUplugs_to_V_idx[plug] #same plug to vector mapping as well, this assumes that the current and voltage plugs are equivalent
            if local_time_index[phase_idx] != np.NaN and ref_time_index[phase_idx] != np.NaN:
                # Extract measurements from closest timestamps
                self.Imag[phase_idx] = local[plug][local_time_index[phase_idx]]['magnitude']
                I_ang_local = local[plug][local_time_index[phase_idx]]['angle']
                I_ang_ref = ref[plug][ref_time_index[phase_idx]]['angle']
                self.Iang[phase_idx] = I_ang_local - I_ang_ref  #uses self. so it defaults to previous value
        return (self.Iang, self.Imag)



    def phasorI_estFromScmd(self, Vmag_relative_pu, Vang, Pcmd_pu, Qcmd_pu):
        dVcomp = Vmag_relative_pu*np.cos(Vang) + Vmag_relative_pu*np.sin(Vang)*1j
        Scomp_est = Pcmd_pu + Qcmd_pu*1j
        Icomp_est = np.conj(Scomp_est/Vcomp) #this will do element_wise computation bc they're arrays
        return (Icomp_est)



    #just uses the most recent current and voltage measurements, doesnt need a match w reference
    def PQ_solver(self, local_phasors, nphases, PMUplugs_to_V_idx):
        # Initialize
        V_mag = [0.0] * nphases
        V_ang = [0.0] * nphases
        I_mag = [0.0] * nphases
        I_ang = [0.0] * nphases
        theta = [0.0] * nphases
        Pact_kVA = [0.0] * nphases
        Qact_kVA = [0.0] * nphases
        for plug in range(nphases):
            phase_idx = PMUplugs_to_V_idx[plug] #assumes plug to V map is the same for uPMUp123 voltage, uPMU123 current and uPMU123 voltage
            V_mag[phase_idx] = local_phasors[nphases*2 + plug][-1]['magnitude'] #pulls out vmeas from uPMU123 not uPMUP123
            V_ang[phase_idx] = local_phasors[nphases*2 + plug][-1]['angle']
            I_mag[phase_idx] = local_phasors[(nphases + plug)][-1]['magnitude']
            I_ang[phase_idx] = local_phasors[(nphases + plug)][-1]['angle']
            theta[phase_idx] = V_ang[phase_idx] - I_ang[phase_idx]
            # P = (VI)cos(theta), Q = (VI)sin(theta)
            Pact_kVA[phase_idx] = V_mag[phase_idx] * I_mag[phase_idx] * (np.cos(np.radians(theta[phase_idx])))/1000
            Qact_kVA[phase_idx] = V_mag[phase_idx] * I_mag[phase_idx] * (np.sin(np.radians(theta[phase_idx])))/1000
        return (Pact_kVA,Qact_kVA)





    def checkSaturation(self, nphases, Pact, Qact, Pcmd, Qcmd):
        "Checking for P saturation (anti-windup control)"
        # find indicies where Pact + tolerance is less than Pcmd
        indexP = np.where(abs(Pact + (0.03 * Pcmd)) < abs(Pcmd))[0] #will be zero if Pcmd is zero
        # initialize saturation counter for each phase
        sat_arrayP = np.ones(nphases) #
        # stop integrator for saturated phases
        for i in indexP:
            sat_arrayP[i] = 0 #0 where saturated
        "Checking for Q saturation (anti-windup control)"
        # find indicies where Qact + tolerance is less than Qcmd
        indexQ = np.where(abs(Qact + (0.03 * Qcmd)) < abs(Qcmd))[0]
        # initialize saturation counter for each phase
        sat_arrayQ = np.ones(nphases)
        # stop integrator for saturated phases
        for i in indexQ:
            sat_arrayQ[i] = 0
        return(sat_arrayP,sat_arrayQ)



    def determineICDI(self, nphases, sat_arrayP, sat_arrayQ, Pact_pu, Qact_pu):
        # saturation counter check to determine if I Cant Do It signal should be sent to SPBC
        self.Psat = np.append(self.Psat, sat_arrayP, axis=1)
        self.Psat = self.Psat[:, 1:] #iterates the Psat counter array to include the new value, discards the old
        for phase in range(nphases):
            if phase in np.where(~self.Psat.any(axis=1))[0]: #if each row doesnt have a 1 in it, then send ICDI for that phase
                ICDI_sigP[phase] = True
                Pmax_pu[phase] = Pact_pu[phase]
            else:
                ICDI_sigP[phase] = False
                Pmax_pu[phase] = np.NaN
        self.Qsat = np.append(self.Qsat, sat_arrayQ, axis=1)
        self.Qsat = self.Qsat[:, 1:]
        for phase in range(nphases):
            if phase in np.where(~self.Qsat.any(axis=1))[0]:
                ICDI_sigQ[phase] = True
                Qmax_pu[phase] = Qact_pu[phase]
            else:
                ICDI_sigQ[phase] = False
                Qmax_pu[phase] = np.NaN
        return (ICDI_sigP, ICDI_sigQ, Pmax_pu, Qmax_pu)





    def httptoInverters(self, nphases, act_idxs, Pcmd_kVA, Qcmd_kVA, Pact):
        # hostname: http://131.243.41.47:
        # port: 9090
        #  Sends P and Q command to actuator
        #needs an up-to-date Pact, which requires a current measurement
        Pcmd_VA = Pcmd_kVA*1000
        Qcmd_VA = Qcmd_kVA*1000
        #initialize parallel API command:
        session = FuturesSession()
        urls = []
        commandReceipt = np.zeros(nphases)
        if self.mode == 1: #1: PV as disturbance
            P_PV = Pact - self.batt_cmd #batt_cmd from last round, still in effect
            for i, inv in zip(range(nphases), act_idxs):
                self.batt_cmd[i] = int(round(Pcmd_VA[i])) #in mode 1 the battery is controlled directly
                if abs(self.batt_cmd[i]) > self.batt_max:
                    self.batt_cmd[i] = int(np.sign(Pcmd_VA) * self.batt_max)
                if ((self.batt_cmd[i] + P_PV)**2 + Qcmd_VA[i]**2) > (self.inv_s_max)**2: #if Qcmd is over the max, set it to the max for the given P command (favors P over Q)
                    Qcmd_VA[i] = np.sign(Qcmd_VA[i]) * np.sqrt((self.inv_s_max)**2 - (self.batt_cmd[i] + P_PV)**2) #what happens by default? it probably maintains the PF command and just produces less P (and the battery curtails itself naturally)
                pf_ctrl = ((np.sign(Qcmd_VA[i]) * -1.0)*abs(self.batt_cmd[i] + P_PV)) / \
                          (np.sqrt(((self.batt_cmd[i] + P_PV)**2) + (Qcmd_VA[i]**2))) #self.batt_cmd[i] + P_PV is ~ the full P flowing through the inverter
                urls.append(f"http://131.243.41.47:9090/control?inv_id={inv[0]},Batt_ctrl={self.batt_cmd[i][0]},"
                              f"pf_ctrl={pf_ctrl[0]}")
        if self.mode == 2: #mode 2: PV calculated
            P_PV = Pact - self.batt_cmd #batt_cmd from last round, still in effect
            for i, inv in zip(range(nphases), act_idxs):
                self.batt_cmd[i] = int(round(Pcmd_VA[i] - P_PV[i])) #in mode 2 the battery and PV are controlled jointly
                if abs(self.batt_cmd[i]) > self.batt_max:
                    self.batt_cmd[i] = int(np.sign(Pcmd_VA) * self.batt_max)
                if (self.batt_cmd[i]**2 + Qcmd_VA[i]**2) > (self.inv_s_max)**2: #if Qcmd is over the max, set it to the max for the given P command (favors P over Q)
                    Qcmd_VA[i] = np.sign(Qcmd_VA[i]) * np.sqrt((self.inv_s_max)**2 - self.batt_cmd[i]**2)
                pf_ctrl = ((np.sign(Qcmd_VA[i]) * -1.0)*abs(self.batt_cmd[i])) / \
                          (np.sqrt((self.batt_cmd[i]**2) + (Qcmd_VA[i]**2))) #self.batt_cmd is ~ the full P flowing through the inverter
                urls.append(f"http://131.243.41.47:9090/control?inv_id={inv[0]},Batt_ctrl={self.batt_cmd[i][0]},"
                              f"pf_ctrl={pf_ctrl[0]}")
        if self.mode == 3: #mode 3: PV only
            for i, inv in zip(range(nphases), act_idxs): #HERE make sure act_idxs is working
                Inv_Pperc_max = 97
                #in mode 3 p_ctrl is used instead of battery control, to control PV
                if Pcmd_VA[i] < 0:
                    Pcmd_VA[i] = 0
                self.invPperc_ctrl[i] = (Pcmd_VA[i] / self.inv_s_max) * 100 #invPperc_ctrl cannot be negative
                if self.invPperc_ctrl[i] > Inv_Pperc_max:
                    self.invPperc_ctrl[i] = Inv_Pperc_max
                    pf_ctrl = 1
                    # pf_ctrl = ((np.sign(Qcmd_VA[i]) * -1.0) * Inv_Pperc_max
                else:
                    pf_ctrl = ((np.sign(Qcmd_VA[i]) * -1.0) *abs(Pcmd_VA[i])) / \
                              (np.sqrt((Pcmd_VA[i] ** 2) + (Qcmd_VA[i] ** 2)))
                urls.append(f"http://131.243.41.47:9090/control?inv_id={inv[0]},P_ctrl={self.invPperc_ctrl[i][0]},"
                              f"pf_ctrl={pf_ctrl[0]}")
        responses = map(session.get, urls)
        results = [resp.result() for resp in responses]
        for i in range(nphases):
            if results[i].status_code == 200:
                commandReceipt[i] = 'success'
            else:
                commandReceipt[i] = 'failure'
        return commandReceipt



    def httptoLoads(self, act_idxs, Pcmd_kVA, Qcmd_kVA):
        #load commands are between 0 and 2000, but from the LPBC's perspective it can control between -1000 and 1000 W, with 1000 W collocated
        Pcmd_VA = Pcmd_kVA*1000
        Qcmd_VA = Qcmd_kVA*1000
        #initialize parallel API command:
        session = FuturesSession()
        urls = []
        commandReceipt = np.zeros(nphases)
        for i, group in zip(range(nphases), act_idxs):
            self.load_cmd[i] = int(np.round((-1. * Pcmd_VA[i]) + self.loadrackPlimit/2)) # -1* bc command goes to a load not an inverter, +self.loadrackPlimit/2 centers the command around 0
            if self.load_cmd[i] > self.loadrackPlimit:
                urls.append(f"http://131.243.41.118:9090/control?group_id={group[0]},P_ctrl=2000")
            elif self.load_cmd[i] < 0:
                urls.append(f"http://131.243.41.118:9090/control?group_id={group[0]},P_ctrl=0")
            else:
                urls.append(f"http://131.243.41.118:9090/control?group_id={group[0]},P_ctrl={self.load_cmd[phase][0]}")
        responses = map(session.get, urls)
        results = [resp.result() for resp in responses]
        for i in range(nphases):
            if results[i].status_code == 200:
                commandReceipt[i] = 'success'
            else:
                commandReceipt[i] = 'failure'
        return commandReceipt




    def initializeActuators(self, mode):
        if mode == 1 or mode == 2:
            responseInverters = requests.get("http://131.243.41.47:9090/control?P_ctrl=97,Batt_ctrl=0")
        if mode == 3:
            responseInverters = requests.get("http://131.243.41.47:9090/control?P_ctrl=0,Batt_ctrl=0")
        responseLoads = requests.get(f"http://131.243.41.118:9090/control?P_ctrl=0")
        if responseInverters.status_code != 200 or responseLoads.status_code != 200:
            error('Error with actuator initialization, responseInverters.status_code = ' + str(responseInverters.status_code) + 'responseLoads.status_code = ' + str(responseLoads.status_code))
        return (responseInverters.status_code, responseLoads.status_code)




    def statusforSPBC(self, PMUplugs_to_phase_map, channels, phasor_error_mag_pu, phasor_error_ang, ICDI_sigP, ICDI_sigQ, Pmax_pu, Qmax_pu):
        status = {}
        # status['phases'] = PMUplugs_to_phase_map #This has to be a list (cant just be a string) #HERE may create an error cause its a numpy array currently
        status['phasor_errors'] = {
                'V': list(phasor_error_mag_pu.ravel()), #ravel flatens the dimensions
                'delta': list(phasor_error_ang.ravel())
            }
        status['p_saturated'] = list(ICDI_sigP.ravel())
        status['q_saturated'] = list(ICDI_sigQ.ravel())
        status['p_max'] = list(Pmax_pu.ravel())
        status['q_max'] = list(Qmax_pu.ravel())
        return(status)




    #step gets called every (rate) seconds starting with init in LPBCProcess within do_trigger/trigger/call_periodic (XBOSProcess) with:
    #status = self.step(local_phasors, reference_phasors, phasor_targets)
    def step(self, local_phasors, reference_phasors, phasor_target): #HERE what happens when no PMU readings are given (Gabe), maybe step wont be called
        self.iteration_counter += 1

        #Initilizes actuators, makes sure you're getting through to them
        if self.iteration_counter == 1:
            (responseInverters, responseLoads) = initializeActuators(self.mode) #throws an error if initialization fails

        if phasor_target is None and self.VangTarg == 'initialize':
            print("Iteration", self.iteration_counter, ": No target received by SPBC")
            return #don't need to return a status, when there isnt one to report
        else:
            if phasor_target is None:
                print("Iteration", self.iteration_counter, ": No target received by SPBC: Using last received target")

            else:
                #get targets and bases from phasor_target, sent by the SPBC
                #values are ordered as: A,B,C according to availability, using the names given to the targets (by the SPBC)
                (self.VmagTarg, self.VangTarg, self.kVbase, self.network_kVAbase) = targetExtraction(phasor_target)
                #VmagTarg is given as VmagTarg_relative rn from the SPBC
                #phasor_target is (perLPBC) data packet from SPBC that contains channels (will be phases once fixed), V, delta, kvbase and kvabase
                self.localkVbase = self.kVbase/self.localVratio
                self.localkVAbase = self.network_kVAbase/self.localSratio
                self.localIbase = self.localkVAbase/self.localkVbase
                self.VmagTarg_pu = self.VmagTarg/(self.localkVbase * 1000)

            # calculate relative voltage phasor
            #the correct PMUs for voltage and current (ie uPMUP123 and uPMU123) are linked in the configuration phase, so local_phasors are what you want (already)
            #values are ordered as: A,B,C according to availability, using self.PMUplugs_to_phase_map
            (self.Vang,self.Vmag,self.VmagRef,self.Vmag_relative, local_time_index, ref_time_index, dataWindowLength) = phasorV_calc(local_phasors, reference_phasors, self.nphases, self.PMUplugs_to_V_idx)
            self.Vmag_pu = self.Vmag / (self.localkVbase * 1000)
            self.Vmag_relative_pu = self.Vmag_relative / (self.localkVbase * 1000) #this and the VmagTarg_relative_pu line divides Vmag_ref by self.localkVbase which may create an issue bc Vref != 1.0pu, but thats okay
            # self.VmagTarg_relative = self.VmagTarg - self.VmagRef
            self.VmagTarg_relative = self.VmagTarg #VmagTarg is given as VmagTarg_relative rn from the SPBC
            self.VmagTarg_relative_pu = self.VmagTarg_relative / (self.localkVbase * 1000)
            self.phasor_error_ang = self.VangTarg - self.Vang
            self.phasor_error_mag_pu = self.VmagTarg_relative_pu - self.Vmag_relative_pu
            # self.phasor_error_mag_pu = self.VmagTarg_pu - self.Vmag_pu    #both this and taking the difference of the relative values work

            #get current measurements, determine saturation if current measurements exist
            if self.currentMeasExists:
                (self.Iang,self.Imag) = phasorI_calc(local_time_index, ref_time_index, dataWindowLength, local_phasors, reference_phasors, self.nphases, self.PMUplugs_to_V_idx)
                self.Imag_pu = self.Imag / self.localIbase
                (self.Pact,self.Qact) = PQ_solver(local_phasors, self.nphases, self.PMUplugs_to_V_idx) #calculate P/Q from actuators
                self.Pact_pu = self.Pact / self.localkVAbase
                self.Qact_pu = self.Qact / self.localkVAbase
                (self.sat_arrayP,self.sat_arrayQ) = checkSaturation(self.nphases, self.Pact, self.Qact, self.Pcmd_kVA, self.Qcmd_kVA) #returns vectors that are one where unsaturated and zero where saturated, will be unsaturated with initial Pcmd = Qcmd = 0
                (self.ICDI_sigP, self.ICDI_sigQ, self.Pmax_pu, self.Qmax_pu) = determineICDI(self.nphases, self.sat_arrayP, self.sat_arrayQ, self.Pact_pu, self.Qact_pu)
            else:
                self.Icomp_est = phasorI_estFromScmd(self.Vmag_relative_pu, self.Vang, self.Pcmd_pu, self.Qcmd_pu) #this estimate should be valid even if there are other loads on the LPBC node (as long as the loads are uncorrelated with the commands)
                self.Icomp_pu_est = self.Icomp_est / self.localIbase
            if np.sum(self.sat_arrayP) + np.sum(self.sat_arrayQ) > 0: #these are 1 if unsaturated
                self.saturated = 0
            else:
                self.saturated = 1
            #if saturated, Iang and Imag from measurements are still legit, but Icomp_est is not legit because the s commanded was not actually enacted

            #run control loop
            (self.Pcmd_pu,self.Qcmd_pu) = self.controller.PIiteration(self.phasor_error_mag_pu, self.phasor_error_ang, self.sat_arrayP, self.sat_arrayQ)
            self.Pcmd_kVA = self.Pcmd_pu * self.localkVAbase
            self.Qcmd_kVA = self.Qcmd_pu * self.localkVAbase

            if self.actType == 'inverter':
                if self.currentMeasExists or self.mode == 3:
                    self.commandReceipt = httptoInverters(self.nphases, self.act_idxs, self.Pcmd_kVA, self.Qcmd_kVA, self.Pact) #calculating Pact requires an active current measurement
                else:
                    disp('couldnt send inverter commands because no current measurement available')
            elif self.actType == 'load':
                self.commandReceipt = httptoLoads(self.nphases, self.act_idxs, self.Pcmd_kVA, self.Qcmd_kVA)
            else:
                error('actType error')

            status = statusforSPBC(self.PMUplugs_to_phase_map, self.phasor_error_mag_pu, self.phasor_error_ang, self.ICDI_sigP, self.ICDI_sigQ, self.Pmax_pu, self.Qmax_pu)

            return status






testcase = '37'
testcase = '13unb'
testcase = '13bal'

'''
PMUplugs_to_phase_dict:
not Flexlab-specific dictionary (generally useful).
Creates a dictionary which is keyed by the bus ID, and the entry is a 3-entry or 6-entry list
which maps the PMU channels [L1, L2, L3] or [L1, L2, L3, C1, C2, C3] to the true phases
['A','B','C'] or ['A','B','C','A','B','C']. It is assumed that voltage comes first.
If some phases are not connected, the empty string '' is placed in the location.

act_logicvect_dict:
Flexlab-specific dictionary.
Tells each LPBC which actuator is sends commands to (eg inverter 1 or loads 1-3)

actType_dict:
Flexlab-specific dictionary.
Tells each LPBC whether it sends commands to loads or inverters
'''
PMUplugs_to_phase_dict = dict()
act_logicvect_dict = dict()
actType_dict = dict()

#Test Case
if testcase == '37':
    subkVAbase = 2500
elif testcase == '13unb':
    subkVAbase = 5000
    lpbcidx = ['671','680']
    key = '671'
    PMUplugs_to_phase_dict[key] = np.asarray(['A','B','C'])
    act_logicvect_dict[key] = np.asarray([1 1 1])
    actType_dict[key] = 'inverter'
    key = '680'
    PMUplugs_to_phase_dict[key] = np.asarray(['','','C'])
    act_logicvect_dict[key] = np.asarray([0 0 1])
    actType_dict[key] = 'load'
elif testcase == '13bal':
    subkVAbase = 5000
    lpbcidx = ['675'] #may have to set these manually
    for key in lpbcidx:
        PMUplugs_to_phase_dict[key] = np.asarray(['A','B','C']) #3 phase default #['A','',''] or ['','C',''] or ['A','B','C','A','B','C'] or ['A','','','A','',''] are also examples
        act_logicvect_dict[key] = np.asarray([1 1 1]) #logic vector for 3 actuators (inverters or load racks) on a phase, 1 if connected, 0 if not
        actType_dict[key] = 'inverter' #'inverter' or 'load'
else:

#entity corresponds to a given piece of hardware (eg a server), putting multiple entities so that the lpbcs could go on different pieces of hardware
#these entity files are on the server (Leo)
entitydict = dict()
entitydict[0] = lpbc_1.ent
entitydict[1] = lpbc_2.ent
entitydict[2] = lpbc_3.ent
entitydict[3] = lpbc_4.ent
entitydict[4] = lpbc_5.ent
entitydict[5] = lpbc_6.ent

"Make sure phases are in consecutive order in config. Voltage first, then current. i.e., L1, L2, I1, I2"
pmu123Channels = ['uPMU_123/L1','uPMU_123/L2','uPMU_123/L3','uPMU_123/C1','uPMU_123/C2','uPMU_123/C3']
pmu123PChannels = ['uPMU_123P/L1','uPMU_123P/L2','uPMU_123P/L3'] #these also have current channels, but dont need them
pmu4Channels = ['uPMU_4/L1','uPMU_4/L2','uPMU_4/L3']
refChannels = ['uPMU_0/L1','uPMU_0/L2','uPMU_0/L3']

nlpbc = len(lpbcidx)

#cfg file is used to build each LPBC, this is a template that is modified below for each LPBC
cfg_file_template = config_from_file(template.toml) #config_from_file defined in XBOSProcess

lpbcdict = dict()
lpbcCounter = 0
channelCounter = 0 #hack to create channels list for lpbcwrapper to listen to,
#this is only used for setting up the experiment, and thus is totally seperate from phase_counter and channel to phase map which are actually part of the LPBC code
#the way channelCounter is used, the inverter and load rack PMU channels have to be assigned in order (eg L1 then L2 then L3 or [L1, L2, C1, C2] then [L3,C3])
for key in lpbcidx:
    #kVbase = np.NaN #should get this from the SPBC so lpbcwrapper doesnt have to run feeder (which requires networkx)
    #kVAbase = subkVAbase #this should also come from SPBC, once it does you can take it out from here
    nphases = sum(phases[lpbcCounter])
    act_idxs = np.nonzero(act_logicvect_dict[key])
    actType = actType_dict[key]
    PMUchannels_to_phase_Map = PMUplugs_to_phase_dict[key] #this is only 3-long, assumes all PMU channels align
    cfg = cfg_file_template
    # namespace is the account that controls permissions
    cfg['name'] = key
    cfg['entity'] = entitydict[lpbccounter] #entity is like a key for each LPBC
    if channelCounter + nphases > 3:
        error('too many phases')
    else:
        if actType == 'inverter':
            cfg['rate'] = 4
            cfg['local_channels'] = pmu123PChannels[channelCounter:channelCounter+nphases] + pmu123Channels[3 + channelCounter:channelCounter+nphases] + pmu123Channels[channelCounter:channelCounter+nphases] #PMU123 voltages at the end (used in power calc)
            #takes voltage measurements from PMU123P, current from PMU123, voltage measurements from PMU123P
            cfg['reference_channels'] = refChannels[channelCounter:channelCounter+nphases]
            currentMeasExists = True
        elif actType == 'load':
            cfg['rate'] = 4
            cfg['local_channels'] = pmu4Channels[channelCounter:channelCounter+nphases]
            cfg['reference_channels'] = refChannels[channelCounter:channelCounter+nphases]
            currentMeasExists = False
        else:
            error('actType Error')
    cfg['spbc'] = 'spbc-jasper-1'
    lpbcdict[key] = lpbcwrapper(cfg, nphases, act_idxs, actType, PMUchannels_to_phase_Map, currentMeasExists)
    lpbcCounter += 1
    channelCounter += nphases-1
    if channelCounter == 2:
        channelCounter = 0

run_loop() #defined in XBOSProcess




# if len(sys.argv) > 1: #when lpbcwrapper (from terminal) you also have to give the config file
#     cfg = config_from_file(sys.argv[1]) # just does: return toml.load(open(filename)), sys.argv[1] is the second command in the command line
# else:
#     sys.exit("Must supply config file as argument: python3 lpbc.py <config file.toml>")
# lpbc1 = lpbcwrapper(cfg)
# run_loop()

'''
my note of how Gabe's code works:
lpbcwrapper is an LPBCProcess which is an XBOSProcess https://github.com/gtfierro/xboswave/blob/master/python/pyxbos/pyxbos/process.py
Which has a function call_periodic which calls trigger
LPBCprocess has trigger which calls do_trigger which calls step with local_phasors, reference_phasors and phasor_targets
Program runs until program is closed in terminal by the user (so it will keep trying to send comands after the simulation ends)



from: https://github.com/Leocchu/energise-implementation/blob/master/LPBC/lpbc.txt
1. Extract and sort phasor target data
2. Call on phasor_calc to determine relative phasor (angle)s
3. Calculate phasor (error)s in per unit quantity
4. Call on PQ_solver to determine P/Q from actuators from PMU current measurement
5. Compare P/Q from inverters with P/Q command sent. If P/Q from inverters is less than P/Q command, then the
inverter is assumed to be saturated and integral action is halted.
6. The saturation counter, self.Psat and self.Qsat, is initialized by user to be of length n. Once saturation has
occurred after n consecutive times, ICDI_sigP and ICDI_sigQ (boolean values), will be sent back to SPBC. If an
ICDI signal is sent, a P/Q max value, self.Pmax or self.Qmax, will be sent also. The P/Q max values will be the
P/Q values generated from PQ_solver (i.e., the P/Q of the inverters when saturated).
7. PI control algorithm using the phasor error and saturation condition. Control algorithm is calculated using per
unit values
8. Convert per unit values in actual values (W/ VARs).
9. Calculate the P/Q commands as a percentage of the inverter apparent power limit and power factor.
10. Send P/Q commands via http. If P/Q commands exceed inverter limit, commands will be truncated to inverter max.
11. 'status' dict containing phases, phasor errors, P/Q ICDI signals, P/Q max is sent back to SPBC for evaluation.
'status' is a dict where the value of the field is a list of numpy data type arguments (except for 'phases'
which is a native python string). For example, calling:
"status['p_max']" will return: "[12.3]" for single-phase, and "[12.3, 15.3, 19.3]" for 3-phase.
"status['p_saturated'] will return: "[False]" for single-phase, and "[False, False, False]" for 3-phase.



from: https://github.com/gtfierro/energise-implementation/blob/master/LPBC/lpbc-example.py
        """
        Step is called every 'rate' seconds with the following data:
        - local_phasors: a list of lists of phasor data, corresponding to the
          'local_channels' given in the LPBC configuration. The phasor data will
          contain *all* phasor data received by the LPBC since the last time the
          'step' function was run. The outer list of local phasor channels is ordered
          the same as the 'local_channels' configuration variable.
          If 'local_channels=["L1","L2"]', then 'local_phasors' will look like
            [
                # data for L1
                [
                    {
                        "time": "1559231114799996800",
                        "angle": 193.30149788923268,
                        "magnitude": 0.038565948605537415
                    },
                    {
                        "time": "1559231114899996400",
                        "angle": 195.50249902851263,
                        "magnitude": 0.042079225182533264
                    }
                ],
                # data for L2
                [
                    {
                        "time": "1559231114799996800",
                        "angle": 193.30149788923268,
                        "magnitude": 0.038565948605537415
                    },
                    {
                        "time": "1559231114899996400",
                        "angle": 195.50249902851263,
                        "magnitude": 0.042079225182533264
                    }
                ],
            ]
        - reference_phasors: a list of lists of phasor data, corresponding to the
          'reference_channels' given in the LPBC configuration. The phasor data will
          contain *all* phasor data received by the LPBC since the last time the
          'step' function was run. The outer list of reference phasor channels is ordered
          the same as the 'reference_channels' configuration variable.
          The structure of the 'reference_phasors' is the same structure as 'local_phasors' above.
        - phasor_target: is the most recently received phasor target given by the SPBC.
          The phasor target key is an array of the targets for each phase.
          It is structured as follows:
            {
                'time': "1559231114799996800", # SPBC time in nanoseconds
                'phasor_targets': [
                    {
                        'nodeID': <lpbc name>,
                        'channelName': 'L1',
                        'angle': 196.123,
                        'magnitude': 10.2,
                        'kvbase': {'value': 10},
                    },
                    {
                        'nodeID': <lpbc name>,
                        'channelName': 'L2',
                        'angle': 196.123,
                        'magnitude': 10.2,
                        'kvbase': {'value': 10},
                    },
                ]
            }
        """
'''
