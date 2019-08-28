

from pyxbos.process import run_loop, config_from_file #https://github.com/gtfierro/xboswave/blob/master/python/pyxbos/pyxbos/process.py
from pyxbos.drivers import pbc  #https://github.com/gtfierro/xboswave/tree/master/python/pyxbos/pyxbos/drivers
# above imports LPBCProcess, SPBCProcess, EnergiseMessage, LPBCStatus, LPBCCommand, SPBC, EnergiseError
import sys
import numpy as np
import warnings
import logging
import requests
from requests_futures.sessions import FuturesSession
from pymodbus.client.sync import ModbusTcpClient as ModbusClient

warnings.simplefilter(action='ignore', category=FutureWarning)
logging.basicConfig(level="INFO", format='%(asctime)s - %(name)s - %(message)s')

from PIcontroller import *
#from APC import *

import time
import datetime as dt
import pytz
import pandas as pd

## PATH NAME OF CSV ##

#to use session.get for parallel API commands you have to download futures: pip install --user requests-futures


class lpbcwrapper(pbc.LPBCProcess): #this is related to super(), inherits attributes and behaviors from pbc.LPBCProcess (which is a wrapper for XBOSProcess)
    def __init__(self, cfg, busId, nphases, act_idxs, actType, plug_to_phase_idx, currentMeasExists, localSratio=1, localVratio=1, ORT_max_kVA = 350):
        super().__init__(cfg)

        # INITIALIZATION
        self.busId = busId #NEW

        #HERE put in optional accumulator term for PI controller

        #NEW
        #get ZskestIinit from excel file somewhere, cause LPBCwrapper doesnt know feeder (or have netowrkx)
        # Zskestinit = (from toml file or whatever, shoudl be nphases-long)
        # Qcost = np.eye(nphases*4)
        # Rcost = np.eye(nphases*2)*10^-1 # for LQR controller
        # self.controller = APCcontroller(nphases,busID,timesteplength,Qcost,Rcost,VmagRef,VangRef,controllerUpdateCadence,saturated,lpAlpha,linearizeplant,lam,ninit,ZskAsMatrix,Zskinit,Gt):
        #TODO add different controllers here: set controller to 'PI' or 'LQR'

        #HERE JASPER!        TO INIT: responseInverters = requests.get("http://131.243.41.47:9090/control?P_ctrl=97,Batt_ctrl=0")
        self.Pcmd_kVA = [0, 1, 2, 3.3]
        self.Pcmd_kVA_t2 = [0, -1, -2, -3.3]
        self.Qcmd_kVA = [0, .1, .2, .3, .4]
        self.Qcmd_kVA_t2 = [0, -.1, -.2, -.3, -.4]
        self.q_count = 0
        self.test = 3.1 # see picture on white board (1, 2, 3.1, 3.2)
        self.mode = 1 #Howe we control inverters mode 1: PV as disturbance, mode 2: PV calculated as act, mode 3: PV only
        print(self.test)
        
        #empty lists for storing and writing to csv
        self.cmd_epoch =[]
        self.Pcmd_store = []
        self.Qcmd_store = []
        self.P_act_store = []
        self.Q_act_store = []
        self.P_PV_store = []
        self.test_phase_shift = []
        self.inv_time = []
        # UP THERE JASPER!

        self.actType = actType
        self.nphases = nphases #TODO
        self.iteration_counter = 0
        self.ametek_phase_shift = 0
        #self.Qcmd_kVA = np.zeros(nphases) #Pcmd comes from the feedback controller
        self.Pcmd_pu = np.zeros(nphases)  #both Pcmd and Pact are in the local power setting (not the amplified OpalRT setting which is just multiplied by localSratio)
        self.Qcmd_pu = np.zeros(nphases)

        self.Pact_kVA = np.zeros(nphases) #Pactual (measured from pmus, used to calc saturation)
        self.Qact_kVA = np.zeros(nphases)
        self.Pact = np.zeros(nphases)
        self.Qact = np.zeros(nphases)
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
        self.status_phases = []

        self.kVbase = np.NaN #received from SPBC #intialized the first time a phasor_target packet comes from the SPBC, control loop isnt run until a packet is received
        self.localVratio = localVratio #!= 1 if Ametek voltage ratio needs to be taken into account (ie PMU123 not PMUP123 used for the voltage)
        self.localkVbase = np.NaN # = self.kVbase/self.localVratio
        self.network_kVAbase = np.NaN #received from SPBC
        self.localSratio = localSratio #ratio between actual power and power in Opal siulation, eg 500/3.3
        self.localkVAbase = np.NaN # = self.network_kVAbase/self.localSratio
        self.localIbase = np.NaN # = self.network_kVAbase/self.localkVbase
        self.ORT_max_VA = ORT_max_kVA * 1000

        # self.plug_to_phase_map = plug_to_phase_map #3-entry vector that maps PMU channels to the true phases (SPBC commands should be given in terms of the true phases)
        self.plug_to_phase_idx = plug_to_phase_idx
        self.plug_to_V_idx = [0] * nphases #maps the plugs (ordered L1 L2 L3) of the PMU to the entries of the V (or I) vectors such that the order is always A then B then C (when each phase is available)
        #if 2 or 3 phase do below: (if its single phase you can just leave it)
        if nphases > 1:
            if 'A' in self.plug_to_phase_idx[:nphases]: #takes care of the 3-phase, and 2 of the 3 2-phase scenarios # dont the :nphases if plug_to_phase_map is 3-long
                for i, phase in enumerate(self.plug_to_phase_idx[:nphases]): #[:nphases] gives the first nphases values (so it takes care of the cases when current measurements are included as well)
                    if phase == 'A':
                        self.plug_to_V_idx[i] = 0 #eg if L2 maps to A then then second entry of self.plug_to_V_idx will be 0
                    if phase == 'B':
                        self.plug_to_V_idx[i] = 1
                    if phase == 'C':
                        self.plug_to_V_idx[i] = 2 - (3 - nphases) #writes a 1 if just A and C
            else:
                for i, phase in enumerate(self.plug_to_phase_idx[:nphases]): #takes care of the case when just B and C phases are present
                    if phase == 'B':
                        self.plug_to_V_idx[i] = 0
                    if phase == 'C':
                        self.plug_to_V_idx[i] = 1

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
 #'inverter' or 'load'
        if self.actType == 'inverter':
            self.act_idxs = self.act_idxs + 1 #inverters indexed starting with 1 not 0

        #Flexlab specific commands
        self.currentMeasExists = currentMeasExists
        self.loadrackPlimit = 2000. #size of a load rack in VA
        self.batt_max = 3300.
        self.inv_s_max = 7600.
        #self.batt_cmd = np.zeros(nphases) #battery commands are given in watts
        self.batt_cmd = np.ones(nphases)
        self.invPperc_ctrl = np.zeros(nphases) #inverter P commnads are given as a percentage of inv_s_max
        self.load_cmd = np.zeros(nphases) #load commands are given in watts


    def targetExtraction(self,phasor_target):
        #this implies A,B,C order to measurements from SPBC
        Vmag_targ_dict = dict()
        Vang_targ_dict = dict()
        kvbase_dict = dict()
        kvabase_dict = dict()
        phaseA = False
        phaseB = False
        phaseC = False
        status_phases = []
        for i in np.arange(nphases):
            if 'ph_A' in phasor_target['phasor_targets'][i]['channelName']:
                phaseA = True
                phase = 'A'
                status_phases.append('ph_A')
            elif 'ph_B' in phasor_target['phasor_targets'][i]['channelName']:
                phaseB = True
                phase = 'B'
                status_phases.append('ph_B')
            elif 'ph_C' in phasor_target['phasor_targets'][i]['channelName']:
                phaseC = True
                phase = 'C'
                status_phases.append('ph_C')
            else:
                disp('no phase found for target ' + str(i) + 'using phase A')
                phaseA = True
                phase = 'A'
                status_phases.append('ph_A')
            Vmag_targ_dict[phase] = phasor_target['phasor_targets'][i]['magnitude']
            Vang_targ_dict[phase] = phasor_target['phasor_targets'][i]['angle']
            kvbase_dict[phase] = phasor_target['phasor_targets'][i]['kvbase']['value']
            kvabase_dict[phase] = phasor_target['phasor_targets'][i]['KVAbase']['value']

        status_phases = sorted(status_phases)
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
        return (Vmag_targ, Vang_targ, kvbase, kvabase, status_phases)
        #there are alternative ways to do this (eg creating Phases_to_V_idx using similar logic)



    def phasorV_calc(self, local_phasors, reference_phasors, nphases, plug_to_V_idx):
        # Initialize
        ordered_local = [0] * nphases # makes a list nphases-long, similar to np.zeros(nphases), but a list
        ref = [0] * nphases
        flag = [1] * nphases #used to check if a phasor match was found
        # Extract latest nPhasorReadings readings from local and ref uPMUs, and put local in phase-order (ref is assumed to be in phase-order)
        for plug in range(nphases): #this will just read the voltage measurements cause its nphases long, even if local_phasors also has current measurements
            if len(local_phasors[plug]) > self.nPhasorReadings:
                dataWindowLength = self.nPhasorReadings
            else:
                dataWindowLength = len(local_phasors[plug])
            phase_idx = plug_to_V_idx[plug]
            ordered_local[phase_idx] = local_phasors[plug][-dataWindowLength:] #this orders local in A,B,C phase order (ref is assumed ot be in A,B,C order)
            ref[plug] = reference_phasors[plug][-dataWindowLength:] #from dataWindowLength back to present, puts Lx2 entries in each entry of local, x2 is for magnitude and phase
        if self.Vang == 'initialize':
            self.Vang = np.zeros(nphases)
            # loops through every phase with actuation
            for phase in range(nphases):
                # Initialize: Extract measurements from most recent timestamps only for first iteration
                V_mag_local = ordered_local[phase][-1]['magnitude']
                V_ang_local = ordered_local[phase][-1]['angle'] - self.ametek_phase_shift
                V_mag_ref = ref[phase][-1]['magnitude']
                V_ang_ref = ref[phase][-1]['angle']
                self.Vang[phase] = V_ang_local - V_ang_ref
                self.Vmag[phase] = V_mag_local
                self.VmagRef[phase] = V_mag_ref
                self.Vmag_relative[phase] = V_mag_local - V_mag_ref
        # loops through each set of voltage measurements for each phase
        local_time_index = [np.NaN]*nphases
        ref_time_index = [np.NaN]*nphases
        for phase in range(nphases): #plug is the number of the PMU plug (0, 1 or 2)
            # loops through every ordered_local uPMU reading starting from most recent
            for local_packet in reversed(ordered_local[phase]):
                # extract most recent ordered_local uPMU reading
                local_time = int(local_packet['time'])
                # loops though every reference uPMU reading starting from most recent
                for ref_packet in reversed(ref[phase]):
                    ref_time = int(ref_packet['time'])
                    # check timestamps of ordered_local and reference uPMU if within 2 ms
                    if abs(ref_time - local_time) <= self.pmuTimeWindow:
                        local_time_index[phase] = ordered_local[phase].index(local_packet)
                        ref_time_index[phase] = ref[phase].index(ref_packet)
                        # Extract measurements from closest timestamps
                        V_mag_local = ordered_local[phase][local_time_index[phase]]['magnitude']
                        V_ang_local = ordered_local[phase][local_time_index[phase]]['angle'] - self.ametek_phase_shift
                        V_mag_ref = ref[phase][ref_time_index[phase]]['magnitude']
                        V_ang_ref = ref[phase][ref_time_index[phase]]['angle']
                        # calculates relative phasors
                        self.Vang[phase] = V_ang_local - V_ang_ref
                        self.Vmag[phase] = V_mag_local
                        self.VmagRef[phase] = V_mag_ref
                        self.Vmag_relative[phase] = V_mag_local - V_mag_ref
                        flag[phase] = 0
                        break
                if flag[phase] == 0:
                    break
            if flag[phase] == 1:
                print("Phase", phase, ", Iteration ", self.iteration_counter, ": No timestamp found")
        return (self.Vang,self.Vmag,self.VmagRef,self.Vmag_relative, local_time_index, ref_time_index, dataWindowLength) #returns the self. variables bc in case a match isnt found, they're already initialized



    def phasorI_calc(self, local_time_index, ref_time_index, dataWindowLength, local_phasors, reference_phasors, nphases, plug_to_V_idx):
        #uses the same time indeces that were found from the voltage search
        # Initialize
        ordered_local = [0] * nphases # makes a list nphases-long, similar to np.zeros(nphases), but a list
        ref = [0] * nphases
        for plug in range(nphases):
            phase_idx = plug_to_V_idx[plug]
            ordered_local[phase_idx] = local_phasors[plug + nphases][-dataWindowLength:] #from dataWindowLength back to present, puts Lx2 entries in each entry of local, x2 is for magnitude and phase
            ref[plug] = reference_phasors[plug + nphases][-dataWindowLength:] #plug + nphases selects the current data rather than the voltage data
        if self.Iang == 'initialize':
            self.Iang = np.zeros(nphases)
            for phase in range(nphases):
                # Initialize: Extract measurements from most recent timestamps only for first iteration
                self.Imag[phase] = ordered_local[phase][-1]['magnitude']
                I_ang_local = ordered_local[phase][-1]['angle']
                I_ang_ref = ref[phase][-1]['angle']
                self.Iang[phase] = I_ang_local - I_ang_ref
        for phase in range(nphases):
            if local_time_index[phase] != np.NaN and ref_time_index[phase] != np.NaN:
                # Extract measurements from closest timestamps
                self.Imag[phase] = ordered_local[phase][local_time_index[phase]]['magnitude']
                I_ang_local = ordered_local[phase][local_time_index[phase]]['angle']
                I_ang_ref = ref[phase][ref_time_index[phase]]['angle']
                self.Iang[phase] = I_ang_local - I_ang_ref  #uses self. so it defaults to previous value
        return (self.Iang, self.Imag)


    def phasorI_estFromScmd(self, Vmag_relative_pu, Vang, Pcmd_pu, Qcmd_pu):
        Vcomp = Vmag_relative_pu*np.cos(Vang) + Vmag_relative_pu*np.sin(Vang)*1j
        Scomp_est = Pcmd_pu + Qcmd_pu*1j
        Icomp_est = np.conj(Scomp_est/Vcomp) #this will do element_wise computation bc they're arrays
        return (Icomp_est)


    #just uses the most recent current and voltage measurements, doesnt need a match w reference
    def PQ_solver(self, local_phasors, nphases, plug_to_V_idx):
        # Initialize
        V_mag = [0.0] * nphases
        V_ang = [0.0] * nphases
        I_mag = [0.0] * nphases
        I_ang = [0.0] * nphases
        theta = [0.0] * nphases
        Pact_kVA = np.asarray([0.0] * nphases)
        Qact_kVA = np.asarray([0.0] * nphases)
        for plug in range(nphases):
            phase_idx = plug_to_V_idx[plug] #assumes plug to V map is the same for uPMUp123 voltage, uPMU123 current and uPMU123 voltage
            V_mag[phase_idx] = local_phasors[nphases*2 + plug][-1]['magnitude'] #pulls out vmeas from uPMU123 not uPMUP123
            V_ang[phase_idx] = local_phasors[nphases*2 + plug][-1]['angle']
            I_mag[phase_idx] = local_phasors[(nphases + plug)][-1]['magnitude']
            I_ang[phase_idx] = local_phasors[(nphases + plug)][-1]['angle']
            theta[phase_idx] = V_ang[phase_idx] - I_ang[phase_idx]
            # P = (VI)cos(theta), Q = (VI)sin(theta)
            Pact_kVA[phase_idx] = V_mag[phase_idx] * I_mag[phase_idx] * (np.cos(np.radians(theta[phase_idx])))/1000
            Qact_kVA[phase_idx] = V_mag[phase_idx] * I_mag[phase_idx] * (np.sin(np.radians(theta[phase_idx])))/1000
        return (Pact_kVA,Qact_kVA)



    def checkSaturation(self, nphases, Pact, Qact, Pcmd_kVA, Qcmd_kVA,):
        Pcmd = Pcmd_kVA * 1000
        Qcmd = Qcmd_kVA * 1000
        if self.actType == 'inverters':
            # find indicies where Pact + tolerance is less than Pcmd
            indexP = np.where(abs(Pact + (0.03 * Pcmd)) < abs(Pcmd))[0] #will be zero if Pcmd is zero
            # find indicies where Qact + tolerance is less than Qcmd
            indexQ = np.where(abs(Qact + (0.03 * Qcmd)) < abs(Qcmd))[0]

        elif self.actType == 'load':
            indexP = np.where(abs(Pcmd) > self.loadrackPlimit/2)[0]
            indexQ = np.where(abs(Qcmd) > self.loadrackPlimit/2)[0]

        elif self.actType == 'modbus':
            indexP = np.where(abs(Pcmd)== self.ORT_max_VA/self.localSratio)[0]
            indexQ = np.where(abs(Qcmd)== self.ORT_max_VA/self.localSratio)[0]

        else:
            error('actType error')

        "Checking for P saturation (anti-windup control)"
        # initialize saturation counter for each phase
        sat_arrayP = np.ones((nphases,1)) #
        # stop integrator for saturated phases
        for i in indexP:
            sat_arrayP[i] = 0 #0 where saturated
        "Checking for Q saturation (anti-windup control)"
        # initialize saturation counter for each phase
        sat_arrayQ = np.ones((nphases,1))
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
                self.ICDI_sigP[phase] = True
                if self.actType == 'inverters':
                    self.Pmax_pu[phase] = Pact_pu[phase]
                elif self.actType == 'load':
                    self.Pmax_pu[phase] = (self.loadrackPlimit/2 * self.localSratio)/(self.network_kVAbase *1000)
                elif self.actType == 'modbus':
                    self.Pmax_pu[phase] = self.ORT_max_VA /(self.network_kVAbase *1000)
            else:
                self.ICDI_sigP[phase] = False
                self.Pmax_pu[phase] = np.NaN
        self.Qsat = np.append(self.Qsat, sat_arrayQ, axis=1)
        self.Qsat = self.Qsat[:, 1:]
        for phase in range(nphases):
            if phase in np.where(~self.Qsat.any(axis=1))[0]:
                self.ICDI_sigQ[phase] = True
                if self.actType == 'inverters':
                    self.Qmax_pu[phase] = Qact_pu[phase]
                elif self.actType == 'load':
                    self.Qmax_pu[phase] = (self.loadrackPlimit/2 * self.localSratio)/(self.network_kVAbase *1000)
                elif self.actType == 'modbus':
                    self.Qmax_pu[phase] = self.ORT_max_VA /(self.network_kVAbase *1000)
            else:
                self.ICDI_sigQ[phase] = False
                self.Qmax_pu[phase] = np.NaN
        return (self.ICDI_sigP, self.ICDI_sigQ, self.Pmax_pu, self.Qmax_pu)



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
        commandReceipt = np.zeros(nphases).tolist()
        if self.mode == 1: #1: PV as disturbance
            print('Q',Qcmd_VA,' P',Pcmd_VA)
            P_PV = Pact*1000 - self.batt_cmd #batt_cmd from last round, still in effect
            print(f'PV: {P_PV}')
            self.P_PV_store.append(P_PV)
            for i, inv in zip(range(nphases), act_idxs):
                self.batt_cmd[i] = int(round(Pcmd_VA)) #in mode 1 the battery is controlled directly
                if abs(self.batt_cmd[i]) > self.batt_max:
                    self.batt_cmd[i] = int(np.sign(Pcmd_VA) * self.batt_max)
                if ((self.batt_cmd[i] + P_PV[i])**2 + Qcmd_VA**2) > (self.inv_s_max)**2: #if Qcmd is over the max, set it to the max for the given P command (favors P over Q)
                    Qcmd_VA = np.sign(Qcmd_VA) * np.sqrt((self.inv_s_max)**2 - (self.batt_cmd[i] + P_PV[i])**2) #what happens by default? it probably maintains the PF command and just produces less P (and the battery curtails itself naturally)
                pf_ctrl = ((np.sign(Qcmd_VA) * -1.0)*abs(self.batt_cmd[i] + P_PV[i])) / \
                          (np.sqrt(((self.batt_cmd[i] + P_PV[i])**2) + (Qcmd_VA**2))) #self.batt_cmd[i] + P_PV is ~ the full P flowing through the inverter
                if self.test == 1 or self.test == 2:
                    pf_ctrl = 1
                #urls.append(f"http://131.243.41.47:9090/control?inv_id={inv},Batt_ctrl={self.batt_cmd[i]},"
                #              f"pf_ctrl={pf_ctrl}")
                if np.abs(pf_ctrl) < 0.01:
                    pf_ctrl = 1
                print(f'pf cmd: {pf_ctrl}')
                urls.append(f"http://131.243.41.47:9090/control?Batt_ctrl={self.batt_cmd[i]},pf_ctrl={pf_ctrl},inv_id={inv}")
        if self.mode == 2: #mode 2: PV calculated
            '''
            P_PV = Pact - self.batt_cmd #batt_cmd from last round, still in effect
            self.P_PV_store.append(P_PV)
            for i, inv in zip(range(nphases), act_idxs):
                self.batt_cmd[i] = int(round(Pcmd_VA[i] - P_PV[i])) #in mode 2 the battery and PV are controlled jointly
                if abs(self.batt_cmd[i]) > self.batt_max:
                    self.batt_cmd[i] = int(np.sign(Pcmd_VA) * self.batt_max)
                if (self.batt_cmd[i]**2 + Qcmd_VA[i]**2) > (self.inv_s_max)**2: #if Qcmd is over the max, set it to the max for the given P command (favors P over Q)
                    Qcmd_VA[i] = np.sign(Qcmd_VA[i]) * np.sqrt((self.inv_s_max)**2 - self.batt_cmd[i]**2)
                pf_ctrl = ((np.sign(Qcmd_VA[i]) * -1.0)*abs(self.batt_cmd[i])) / \
                          (np.sqrt((self.batt_cmd[i]**2) + (Qcmd_VA[i]**2))) #self.batt_cmd is ~ the full P flowing through the inverter
                if self.test == 1 or self.test == 2:
                    pf_ctrl = 1
                    '''
            P_PV = Pact*1000 - self.batt_cmd #batt_cmd from last round, still in effect
            self.P_PV_store.append(P_PV)
            for i, inv in zip(range(nphases), act_idxs):
                self.batt_cmd[i] = int(round(Pcmd_VA - P_PV[i])) #in mode 2 the battery and PV are controlled jointly
                if abs(self.batt_cmd[i]) > self.batt_max:
                    self.batt_cmd[i] = int(np.sign(Pcmd_VA) * self.batt_max)
                if (self.batt_cmd[i]**2 + Qcmd_VA**2) > (self.inv_s_max)**2: #if Qcmd is over the max, set it to the max for the given P command (favors P over Q)
                    Qcmd_VA = np.sign(Qcmd_VA) * np.sqrt((self.inv_s_max)**2 - self.batt_cmd[i]**2)
                pf_ctrl = ((np.sign(Qcmd_VA) * -1.0)*abs(self.batt_cmd[i])) / \
                          (np.sqrt((self.batt_cmd[i]**2) + (Qcmd_VA**2))) #self.batt_cmd is ~ the full P flowing through the inverter
                if self.test == 1 or self.test == 2:
                    pf_ctrl = 1
                if np.abs(pf_ctrl) < 0.01:
                    pf_ctrl = 1                    
                #urls.append(f"http://131.243.41.47:9090/control?inv_id={inv},Batt_ctrl={self.batt_cmd[i]},"
                #              f"pf_ctrl={pf_ctrl}")
                urls.append(f"http://131.243.41.47:9090/control?Batt_ctrl={self.batt_cmd[i]},pf_ctrl={pf_ctrl},inv_id={inv}")
        if self.mode == 3: #mode 3: PV only
            for i, inv in zip(range(nphases), act_idxs): #HERE make sure act_idxs is working
                Inv_Pperc_max = 97
                #in mode 3 p_ctrl is used instead of battery control, to control PV
                if Pcmd_VA < 0:
                    Pcmd_VA = 0
                self.invPperc_ctrl[i] = (Pcmd_VA / self.inv_s_max) * 100 #invPperc_ctrl cannot be negative
                if self.invPperc_ctrl[i] > Inv_Pperc_max:
                    self.invPperc_ctrl[i] = Inv_Pperc_max
                    pf_ctrl = 1
                    # pf_ctrl = ((np.sign(Qcmd_VA[i]) * -1.0) * Inv_Pperc_max
                else:
                    pf_ctrl = ((np.sign(Qcmd_VA) * -1.0) *abs(Pcmd_VA)) / \
                              (np.sqrt((Pcmd_VA ** 2) + (Qcmd_VA ** 2)))
                if self.test == 1 or self.test == 2:
                    pf_ctrl = 1
                if np.abs(pf_ctrl) < 0.01:
                    pf_ctrl = 1
                #urls.append(f"http://131.243.41.47:9090/control?inv_id={inv},P_ctrl={self.invPperc_ctrl[i]},"
                #              f"pf_ctrl={pf_ctrl}")
                urls.append(f"http://131.243.41.47:9090/control?P_ctrl={self.invPperc_ctrl[i]},pf_ctrl={pf_ctrl},inv_id={inv}")
        responses = map(session.get, urls)
        results = [resp.result() for resp in responses]
        for i in range(nphases):
            if results[i].status_code == 200:
                commandReceipt[i] = 'success'
            else:
                commandReceipt[i] = 'failure'
        return commandReceipt


    #step gets called every (rate) seconds starting with init in LPBCProcess within do_trigger/trigger/call_periodic (XBOSProcess) with:
    #status = self.step(local_phasors, reference_phasors, phasor_targets)
    def step(self, local_phasors, reference_phasors, phasor_target):
        print('interation:', self.iteration_counter)
        print('q count:', self.q_count)
        #to change phases see acts_to_phase_dict
        print('length ref:', len(reference_phasors[0]))
        print('length loc:', len(local_phasors[0]))

        if self.test == 1:
            if self.iteration_counter < 5:
                pcmd = self.Pcmd_kVA[0]
                qcmd = 0
                (self.Pact, self.Qact) = self.PQ_solver(local_phasors, self.nphases, self.plug_to_V_idx)
                t = time.time()
                self.cmd_epoch.append(t)
                commandReceipt = self.httptoInverters(self.nphases, self.act_idxs, pcmd, qcmd, self.Pact)
                self.inv_time.append(time.time() - t)
                print('command receipt:',commandReceipt)
                self.P_act_store.append(self.Pact)
                self.Q_act_store.append(self.Qact)
                angle = self.phasorV_calc(local_phasors, reference_phasors, self.nphases, self.plug_to_V_idx)
                self.test_phase_shift.append(angle[0])
                self.Pcmd_store.append(pcmd)
                self.Qcmd_store.append(qcmd)
            elif 4 < self.iteration_counter < 10:
                pcmd = self.Pcmd_kVA[1]
                qcmd = 0
                (self.Pact, self.Qact) = self.PQ_solver(local_phasors, self.nphases, self.plug_to_V_idx)
                t = time.time()
                self.cmd_epoch.append(t)
                commandReceipt = self.httptoInverters(self.nphases, self.act_idxs, pcmd, qcmd, self.Pact)
                self.inv_time.append(time.time() - t)
                print('command receipt:',commandReceipt)
                self.P_act_store.append(self.Pact)
                self.Q_act_store.append(self.Qact)
                angle = self.phasorV_calc(local_phasors, reference_phasors, self.nphases, self.plug_to_V_idx)
                self.test_phase_shift.append(angle[0])
                self.Pcmd_store.append(pcmd)
                self.Qcmd_store.append(qcmd)
            elif 9 < self.iteration_counter < 15:
                pcmd = self.Pcmd_kVA[2]
                qcmd = 0
                (self.Pact, self.Qact) = self.PQ_solver(local_phasors, self.nphases, self.plug_to_V_idx)
                t = time.time()
                self.cmd_epoch.append(t)
                commandReceipt = self.httptoInverters(self.nphases, self.act_idxs, pcmd, qcmd, self.Pact)
                self.inv_time.append(time.time() - t)
                print('command receipt:',commandReceipt)
                self.P_act_store.append(self.Pact)
                self.Q_act_store.append(self.Qact)
                angle = self.phasorV_calc(local_phasors, reference_phasors, self.nphases, self.plug_to_V_idx)
                self.test_phase_shift.append(angle[0])
                self.Pcmd_store.append(pcmd)
                self.Qcmd_store.append(qcmd)
            elif 14 < self.iteration_counter < 20:
                pcmd = self.Pcmd_kVA[3]
                qcmd = 0
                (self.Pact, self.Qact) = self.PQ_solver(local_phasors, self.nphases, self.plug_to_V_idx)
                t = time.time()
                self.cmd_epoch.append(t)
                commandReceipt = self.httptoInverters(self.nphases, self.act_idxs, pcmd, qcmd, self.Pact)
                self.inv_time.append(time.time() - t)
                print('command receipt:',commandReceipt)
                self.P_act_store.append(self.Pact)
                self.Q_act_store.append(self.Qact)
                angle = self.phasorV_calc(local_phasors, reference_phasors, self.nphases, self.plug_to_V_idx)
                self.test_phase_shift.append(angle[0])
                self.Pcmd_store.append(pcmd)
                self.Qcmd_store.append(qcmd)

        if self.test == 2:
            if self.iteration_counter < 5:
                pcmd = self.Pcmd_kVA_t2[0]
                qcmd = 0
                (self.Pact, self.Qact) = self.PQ_solver(local_phasors, self.nphases, self.plug_to_V_idx)                
                t = time.time()
                self.cmd_epoch.append(t)
                commandReceipt = self.httptoInverters(self.nphases, self.act_idxs, pcmd, qcmd, self.Pact)
                self.inv_time.append(time.time() - t)
                print('command receipt:',commandReceipt)
                self.P_act_store.append(self.Pact)
                self.Q_act_store.append(self.Qact)
                angle = self.phasorV_calc(local_phasors, reference_phasors, self.nphases, self.plug_to_V_idx)
                self.test_phase_shift.append(angle[0])
                self.Pcmd_store.append(pcmd)
                self.Qcmd_store.append(qcmd)
            elif 4 < self.iteration_counter < 10:
                pcmd = self.Pcmd_kVA_t2[1]
                qcmd = 0
                (self.Pact, self.Qact) = self.PQ_solver(local_phasors, self.nphases, self.plug_to_V_idx)   
                t = time.time()
                self.cmd_epoch.append(t)
                commandReceipt = self.httptoInverters(self.nphases, self.act_idxs, pcmd, qcmd, self.Pact)
                self.inv_time.append(time.time() - t)
                print('command receipt:',commandReceipt)
                self.P_act_store.append(self.Pact)
                self.Q_act_store.append(self.Qact)
                angle = self.phasorV_calc(local_phasors, reference_phasors, self.nphases, self.plug_to_V_idx)
                self.test_phase_shift.append(angle[0])
                self.Pcmd_store.append(pcmd)
                self.Qcmd_store.append(qcmd)
            elif 9 < self.iteration_counter < 15:
                pcmd = self.Pcmd_kVA_t2[2]
                qcmd = 0
                (self.Pact, self.Qact) = self.PQ_solver(local_phasors, self.nphases, self.plug_to_V_idx)   
                t = time.time()
                self.cmd_epoch.append(t)
                commandReceipt = self.httptoInverters(self.nphases, self.act_idxs, pcmd, qcmd, self.Pact)
                self.inv_time.append(time.time() - t)
                print('command receipt:',commandReceipt)
                self.P_act_store.append(self.Pact)
                self.Q_act_store.append(self.Qact)
                angle = self.phasorV_calc(local_phasors, reference_phasors, self.nphases, self.plug_to_V_idx)
                self.test_phase_shift.append(angle[0])
                self.Pcmd_store.append(pcmd)
                self.Qcmd_store.append(qcmd)
            elif 14 < self.iteration_counter < 20:
                pcmd = self.Pcmd_kVA_t2[3]
                qcmd = 0
                (self.Pact, self.Qact) = self.PQ_solver(local_phasors, self.nphases, self.plug_to_V_idx)   
                t = time.time()
                self.cmd_epoch.append(t)
                commandReceipt = self.httptoInverters(self.nphases, self.act_idxs, pcmd, qcmd, self.Pact)
                self.inv_time.append(time.time() - t)
                print('command receipt:',commandReceipt)
                self.P_act_store.append(self.Pact)
                self.Q_act_store.append(self.Qact)
                angle = self.phasorV_calc(local_phasors, reference_phasors, self.nphases, self.plug_to_V_idx)
                self.test_phase_shift.append(angle[0])
                self.Pcmd_store.append(pcmd)
                self.Qcmd_store.append(qcmd)

        if self.test == 3.1:
            
            if self.iteration_counter < 25:
                pcmd = .4
                qcmd = self.Qcmd_kVA[self.q_count]
                t = time.time()
                self.cmd_epoch.append(t)
                commandReceipt = self.httptoInverters(self.nphases, self.act_idxs, pcmd, self.Qcmd_kVA_t2[self.q_count], self.Pact)
                self.inv_time.append(time.time() - t)
                print('command receipt:',commandReceipt)
                (self.Pact, self.Qact) = self.PQ_solver(local_phasors, self.nphases, self.plug_to_V_idx)
                self.P_act_store.append(self.Pact)
                self.Q_act_store.append(self.Qact)
                self.Pcmd_store.append(pcmd)
                self.Qcmd_store.append(qcmd)
            if self.iteration_counter in np.arange(4,25,5):
                self.q_count += 1
            '''
            if self.iteration_counter == 0 or  self.iteration_counter == 5 or  self.iteration_counter == 10 or self.iteration_counter == 15 or self.iteration_counter == 20:
                pcmd = 200
                qcmd = self.Qcmd_kVA[self.q_count]
                t = time.time()
                self.cmd_epoch.append(t)
                #commandReceipt = self.httptoInverters(self.nphases, self.act_idxs, pcmd, qcmd, self.Pact)
                self.inv_time.append(time.time() - t)
                #print('command receipt:',commandReceipt)
                self.q_count += 1

            if self.iteration_counter < 25:
                pcmd = 200
                qcmd = self.Qcmd_kVA[self.q_count]
                (self.Pact, self.Qact) = self.PQ_solver(local_phasors, self.nphases, self.plug_to_V_idx)
                self.P_act_store.append(self.Pact)
                self.Q_act_store.append(self.Qact)
                self.Pcmd_store.append(pcmd)
                self.Qcmd_store.append(qcmd)
                
                self.cmd_epoch.append('n/a')
                self.inv_time.append('n/a')
            '''

        if self.test == 3.2:
            if self.iteration_counter < 25:
                pcmd = -.4
                qcmd = self.Qcmd_kVA_t2[self.q_count]
                t = time.time()
                self.cmd_epoch.append(t)
                commandReceipt = self.httptoInverters(self.nphases, self.act_idxs, pcmd, self.Qcmd_kVA_t2[self.q_count], self.Pact)
                self.inv_time.append(time.time() - t)
                print('command receipt:',commandReceipt)
                (self.Pact, self.Qact) = self.PQ_solver(local_phasors, self.nphases, self.plug_to_V_idx)
                self.P_act_store.append(self.Pact)
                self.Q_act_store.append(self.Qact)
                self.Pcmd_store.append(pcmd)
                self.Qcmd_store.append(qcmd)
            if self.iteration_counter in np.arange(4,25,5):
                self.q_count += 1

        '''
        #empty lists for storing and writing to csv
        self.cmd_epoch =[]
        self.Pcmd_store = []
        self.Qcmd_store = []
        self.P_act_store = []
        self.Q_act_store = []
        self.P_PV_store = []
        self.test_phase_shift = []
        self.inv_time = []
        # UP THERE JASPER!
        '''
        
        if self.test == 1 or self.test == 2:
            if self.iteration_counter == 19:
                "DO WRITE TO CSV HERE"
                # set path name according to time of creation#
                date = pytz.utc.localize(dt.datetime.utcnow()).astimezone(pytz.timezone('US/Pacific'))
                month, day, hour, minute = date.month, date.day, date.hour, date.minute
                path_to = f'{month}-{day}-{hour}-{minute}_HIL_cal_T{self.test}'
                cmd_datetime = []
                
                # create df
                df = pd.DataFrame()         
                df['cmd_epoch'] = self.cmd_epoch
                for i in df.cmd_epoch:
                    cmd_datetime.append(pytz.utc.localize(dt.datetime.fromtimestamp(i)).astimezone(pytz.timezone('US/Pacific')))
                df['cmd_datetime(PST)'] = cmd_datetime
                #df['phase_shift'] = self.test_phase_shift
                df['Pcmd [kW]'] = self.Pcmd_store
                df['Qcmd [kVAR]'] = self.Qcmd_store
                df['Pact [kW]'] = self.P_act_store
                df['Qact [kVAR]'] = self.Q_act_store
                P_PV_kW_store = [x / 1000 for x in self.P_PV_store]
                df['PV [kW]'] = P_PV_kW_store
                df['inv cmd time'] = self.inv_time
                
                # write df to csv
                df.to_csv('inv_test_results/'+path_to)
                
                print('wrote to csv')
                print('~~~ END ~~~')

        elif self.test == 3.1 or self.test == 3.2:
            if self.iteration_counter == 24:
                "DO WRITE TO CSV HERE"
                # set path name according to time of creation#
                date = pytz.utc.localize(dt.datetime.utcnow()).astimezone(pytz.timezone('US/Pacific'))
                month, day, hour, minute = date.month, date.day, date.hour, date.minute
                path_to = f'{month}-{day}-{hour}-{minute}_HIL_cal_T{str(self.test)[0]}_{str(self.test)[2]}'
                cmd_datetime = []
                
                # create df
                df = pd.DataFrame()         
                df['cmd_epoch'] = self.cmd_epoch
                for i in df.cmd_epoch:
                    cmd_datetime.append(pytz.utc.localize(dt.datetime.fromtimestamp(i)).astimezone(pytz.timezone('US/Pacific')))
                df['cmd_datetime(PST)'] = cmd_datetime
                #df['phase_shift'] = self.test_phase_shift
                df['Pcmd [kW]'] = self.Pcmd_store
                df['Qcmd [kVAR]'] = self.Qcmd_store
                df['Pact [kW]'] = self.P_act_store
                df['Qact [kVAR]'] = self.Q_act_store
                P_PV_kW_store = [x / 1000 for x in self.P_PV_store]
                df['PV [kW]'] = P_PV_kW_store
                df['inv cmd time'] = self.inv_time
                
                # write df to csv
                df.to_csv('inv_test_results/'+path_to)
                
                print('wrote to csv')
                print('~~~ END ~~~')


        self.iteration_counter += 1
        
        return




'''
Network phase, actuator, and pmu plug mapping explanation:

TLDR:
Actuator assignments are determined by acts_to_phase_dict[key] for each simulation.
The entry in acts_to_phase_dict[key] (ie 0, 1 or 2) determines which actuator is controlled.
The phase entry determines which phase, and must (should) coincide with the phases of that node on the simulated network.
Phases must be in order (becasue I havent implemented a phase to idx mapping yet).
The PMU port to actuator mapping is set once for the Flexlab, and applies to all simulations.
The PMU port to actuator mapping is used with the given simulations act to phase mapping to create a port to phase mapping for that simulation (automatically).
That port to phase mapping is sent to LPBC, and is used to order the PMU measurements according to their true phase.

Assumptions:
-Actuators are dispatched in A,B,C order (to ditch this assumption, we would need a phase_to_act map, that might have to come later anyway)
-voltage and current ports of the PMU are attached in the same order
-PMU123 and PMU123P are connected in the same order (because in real applications you will not have a PMU123P)


Longer Explanation:
Want LPBCwrapper to have minimal changes necessary for implementing it outside of FLexlab.
In practice, LPBC is attached to nphases actuators, and has nphases measurements.
LPBC needs to know which phase is which to compare local phasor meas w ref phasor meas (It is assumed that reference phasors come in A,B,C order), and also so the LPBC can maintain phase-specific commands from the SPBC.
So the LPBC needs the channel to phase mapping.
In practice, the LPBC measurements may not go through the WAVE comms network, and the mapping would be plug to phase.
For the Flexlab testing, we assume the plugs to channels are a 1-to-1 mapping

In the Flexlab, there might be a mismatch between the PMU plugs (wave channels) and the actuators.
What we really want is a mapping bn the PMU plugs (wave channels) and the correct phases.
The plug to act mapping has to be determined for the flexlab, but doesnt change.
Each simulation gives the act to phase mapping.
Using both these mappings, we build the plug to phase mapping.
Also, the necessary pmu plugs are put on the network, in the cannonical order (L1,L2,L3 etc)

To preserve the minimal-change-necessary paradigm, it is assumed that PMU123 ang PMU123P are connected the same way,
as well was the voltage and current ports for each PMU.

non-Flexlab-specific dictionaries:

plug_to_phase_dict:
not Flexlab-specific dictionary (generally useful).
Creates a dictionary which is keyed by the bus ID, and the entry is a nplugs-long list
which maps the PMU plugs eg. [L1, L2, L3] to ['A','B','C'] or [L1] to ['B']

Flexlab-specific dictionaries:

acts_to_phase_dict:
Maps the actuators in the flexlab to phases on the network
If some phases are not connected, the empty string '' is placed in the location.

To account for the fact that the PMU plugs and actuators may be misconnected (a flexlab-specific issue),
This code deals with this by using the empty strings '' in acts_to_phase_dict.
If an entry is not empty, then the controller controls that actuator.
The corresponding pmu reading for that actuator is given using the act_to_plug_Map

act_idxs:
Determines which flexlab actuators a given LPBC controls.
assumes that, for each lpbc with multiple actuators, the actuators are dispatched in A, B, C order
also, for each lpbc, there is not more than one actuator on a given phase

actType_dict:
Tells each LPBC whether it sends commands to loads or inverters

'''

acts_to_phase_dict = dict()
actType_dict = dict()

#Manual entry here to determine test case, phases, etc.
#Test Case
#testcase = '37'
#testcase = '13unb'
#testcase = '13bal'
testcase = 'manual'

if testcase == '37':
    subkVAbase = 2500
elif testcase == '13unb':
    subkVAbase = 5000
    lpbcidx = ['671','680']
    key = '671'
    acts_to_phase_dict[key] = np.asarray(['A','B','C']) #phase on the network (in simulation)
    actType_dict[key] = 'inverter'
    key = '680'
    acts_to_phase_dict[key] = np.asarray(['','','C']) #the nonzero entries correspond to the actuator indices
    actType_dict[key] = 'load'
elif testcase == '13bal':
    subkVAbase = 5000
    lpbcidx = ['675'] #may have to set these manually
    for key in lpbcidx: #makes them all three phase inverters
        acts_to_phase_dict[key] = np.asarray(['A','B','C']) #3 phase default #['A','',''] or ['','C',''] or ['A','B','C','A','B','C'] or ['A','','','A','',''] are also examples, ['A','C','B'] and ['B','B','B'] are not allowed (yet)
        actType_dict[key] = 'inverter' #'inverter' or 'load'

#TODO: set test case here
elif testcase == 'manual':
    lpbcidx = ['675'] #nodes of actuation
    key = '675'
    acts_to_phase_dict[key] = np.asarray(['A','','']) #which phases to actuate for each lpbcidx # SET PHASES
    actType_dict[key] = 'inverter' #choose: 'inverters', 'load', or 'modbus'

#these should be established once for the FLexlab,
#they take care of cases where a pmu port does not correspond to the given inverter number
#eg if pmu123 port 2 is attached to inverter 3 and port 3 is attached to inverter 2 pmu123_act_to_plug_Map = np.asarray([0, 2, 1])
pmu0_phase_to_plug_Map = np.asarray([0, 1, 2]) #this is assumed to be true
pmu123_act_to_plug_Map = np.asarray([0, 1, 2])
# pmu123P_act_to_plug_Map = np.asarray([0, 1, 2]) #this is assumed to be in the same order as pmu123_act_to_plug_Map
pmu4_act_to_plug_Map = np.asarray([0, 1, 2])
pmu0_plugs_dict = dict()
pmu123_plugs_dict = dict()
pmu123P_plugs_dict = dict()
pmu4_plugs_dict = dict()

plug_to_phase_dict = dict()

for key in lpbcidx:
    #act_idxs assumes that, for each lpbc with multiple actuators, the actuators are dispatched in A, B, C order
    #also, for each lpbc, there is not more than one actuator on a given phase
    act_idxs = np.nonzero(acts_to_phase_dict[key])[0] #nonzero entries of acts_to_phase_dict[key] are defined as turning on actuators 1, 2, 3, [0] bc np.nonzero() is weird
    nphases = len(act_idxs)

    #In case referenec plugs are not in the connected in the correct A,B,C order:
    #Puts pmu0_plugs_dict[key] in A, B, C order, (assuming XBOS wrapper doesnt take care of this on its own)
    #acts_to_phase_dict[key] has the phases that the reference should listen to (not necessarily in order)
    pmu0_plugs_dict[key] = []
    if 'A' in acts_to_phase_dict[key]:
        pmu0_plugs_dict[key].append(pmu0_phase_to_plug_Map[0]) #if ref needs to listen to A, listen to the PMU plug corresponding to A
    if 'B' in acts_to_phase_dict[key]:
        pmu0_plugs_dict[key].append(pmu0_phase_to_plug_Map[1])
    if 'C' in acts_to_phase_dict[key]:
        pmu0_plugs_dict[key].append(pmu0_phase_to_plug_Map[2])
    pmu0_plugs_dict[key] = np.asarray(pmu0_plugs_dict[key])

    #Does not put local pmus measurements in A, B, C order, but does build plug_to_phase_Map
    #assumes PMU123 and PMU123P are connected in the same order (because in real applications you will not have a PMU123P)
    plug_to_phase_dict[key] = np.asarray(['','',''])
    if actType_dict[key] == 'inverter':
        #puts the the correct PMU measurements into pmu123_plugs_dict[key] and pmu123P_plugs_dict[key] in the order in which the actuators appear in acts_to_phase_dict[key]
        pmu123_plugs_dict[key] = []
        pmu123P_plugs_dict[key] = []
        for i in np.arange(3): #actuator i
            if acts_to_phase_dict[key][i] != '': #if this LPBC uses actuator i
                plug = pmu123_act_to_plug_Map[i] #the pmu plug that corresponds to actuator i
                phase = acts_to_phase_dict[key][i]
                plug_to_phase_dict[key][plug] = phase #places the phase for that wave channel in the plug_to_phase mapping that gets sent to the PMUs
                pmu123_plugs_dict[key].append(plug) #places the PMU measurement corresponding to actuator i on the WAVE channel
                pmu123P_plugs_dict[key].append(plug)
        pmu123_plugs_dict[key] = np.asarray(sorted(pmu123_plugs_dict[key])) #orders the PMU channels in 0,1,2 ordering. This is expected by plug_to_phase_dict, which is sorted implicitly when it is built by inserting into [plug] position (then subsequently reduced to an idx)
        pmu123P_plugs_dict[key] = np.asarray(sorted(pmu123P_plugs_dict[key]))
        print('plugtodict',pmu123_plugs_dict[key])
    elif actType_dict[key] == 'load':
        pmu4_plugs_dict[key] = []
        for i in np.arange(3): #actuator i
            if acts_to_phase_dict[key][i] != '': #if this LPBC uses actuator i
                plug = pmu4_act_to_plug_Map[i] #the pmu plug that corresponds to actuator i
                phase = acts_to_phase_dict[key][i]
                plug_to_phase_dict[key][plug] = phase #places the phase for that wave channel in the plug_to_phase mapping that gets sent to the PMUs
                pmu4_plugs_dict[key].append(plug) #places the PMU measurement corresponding to actuator i on the WAVE channe
        pmu4_plugs_dict[key] = np.asarray(sorted(pmu123_plugs_dict[key])) #orders the PMU channels in 0,1,2 ordering. This is expected by plug_to_phase_dict, which is sorted implicitly
    elif actType_dict[key] == 'modbus':
        pmu123P_plugs_dict[key] = []
        for i in np.arange(3): #actuator i
            if acts_to_phase_dict[key][i] != '': #if this LPBC uses actuator i
                plug = pmu123_act_to_plug_Map[i] #the pmu plug that corresponds to actuator i
                phase = acts_to_phase_dict[key][i]
                plug_to_phase_dict[key][plug] = phase #places the phase for that wave channel in the plug_to_phase mapping that gets sent to the PMUs
                pmu123P_plugs_dict[key].append(plug)
        pmu123P_plugs_dict[key] = np.asarray(sorted(pmu123P_plugs_dict[key]))
#entity corresponds to a given piece of hardware (eg a server), putting multiple entities so that the lpbcs could go on different pieces of hardware
#these entity files are on the server (Leo)

entitydict = dict()
entitydict[0] = 'lpbc_1.ent'
entitydict[1] = 'lpbc_2.ent'
entitydict[2] = 'lpbc_3.ent'
entitydict[3] = 'lpbc_4.ent'
entitydict[4] = 'lpbc_5.ent'
entitydict[5] = 'lpbc_6.ent'

"Make sure phases are in consecutive order in config. Voltage first, then current. i.e., L1, L2, I1, I2"
pmu123Channels = np.asarray(['uPMU_123/L1','uPMU_123/L2','uPMU_123/L3','uPMU_123/C1','uPMU_123/C2','uPMU_123/C3']) #HERE is this okay that this is a numpyarray now? depends on Gabes code, hopefully fine
pmu123PChannels = np.asarray(['uPMU_123P/L1','uPMU_123P/L2','uPMU_123P/L3']) #these also have current channels, but dont need them
pmu4Channels = np.asarray(['uPMU_4/L1','uPMU_4/L2','uPMU_4/L3'])
refChannels = np.asarray(['uPMU_123P/L1','uPMU_123P/L2','uPMU_123P/L3'])

#pmu123Channels = ['uPMU_123/L1','uPMU_123/L2','uPMU_123/L3','uPMU_123/C1','uPMU_123/C2','uPMU_123/C3'] #HERE is this okay that this is a numpyarray now? depends on Gabes code, hopefully fine
#pmu123PChannels = ['uPMU_123P/L1','uPMU_123P/L2','uPMU_123P/L3'] #these also have current channels, but dont need them
#pmu4Channels = ['uPMU_4/L1','uPMU_4/L2','uPMU_4/L3']
#refChannels = ['uPMU_123P/L1','uPMU_123P/L2','uPMU_123P/L3']

nlpbc = len(lpbcidx)

#cfg file is used to build each LPBC, this is a template that is modified below for each LPBC
cfg_file_template = config_from_file('template.toml') #config_from_file defined in XBOSProcess

lpbcdict = dict()
lpbcCounter = 0
for key in lpbcidx:
    #kVbase = np.NaN #should get this from the SPBC so lpbcwrapper doesnt have to run feeder (which requires networkx)
    #kVAbase = subkVAbase #this should also come from SPBC, once it does you can take it out from here
    act_idxs = np.nonzero(acts_to_phase_dict[key])[0]
    nphases = len(act_idxs)
    actType = actType_dict[key]
    plug_to_phase_map = plug_to_phase_dict[key]
    plug_to_phase_idx = plug_to_phase_map[np.nonzero(plug_to_phase_map)]
    cfg = cfg_file_template
    # namespace is the account that controls permissions
    cfg['name'] = key
    cfg['entity'] = entitydict[lpbcCounter] #entity is like a key for each LPBC
    if actType == 'inverter':
        cfg['rate'] = 10 # JASPER CHANGE RATE HERE
        #cfg['local_channels'] = np.concatenate([pmu123Channels[pmu123_plugs_dict[key]], pmu123Channels[3 + pmu123_plugs_dict[key]]])
        cfg['local_channels'] = np.concatenate([pmu123PChannels[pmu123P_plugs_dict[key]], pmu123Channels[3 + pmu123_plugs_dict[key]], pmu123Channels[pmu123_plugs_dict[key]]])
        #takes voltage measurements from PMU123P, current from PMU123, voltage measurements from PMU123P
        cfg['reference_channels'] = refChannels[pmu0_plugs_dict[key]].tolist() #listen to the correct references
        currentMeasExists = True
    elif actType == 'load':
        cfg['rate'] = 4
        cfg['local_channels'] = pmu4Channels[pmu4_plugs_dict[key]]
        cfg['reference_channels'] = refChannels[pmu0_plugs_dict[key]]
        currentMeasExists = False
    elif actType == 'modbus':
        cfg['rate'] = 5
        cfg['local_channels'] = pmu123PChannels[pmu123P_plugs_dict[key]]
        cfg['reference_channels'] = refChannels[pmu0_plugs_dict[key]]
        currentMeasExists = False
    else:
        error('actType Error')
    cfg['spbc'] = 'spbc-jasper-1'
    lpbcdict[key] = lpbcwrapper(cfg, key, nphases, act_idxs, actType, plug_to_phase_idx, currentMeasExists) #Every LPBC will have its own step that it calls on its own
    lpbcCounter += 1

run_loop() #defined in XBOSProcess
