

from pyxbos.process import run_loop, config_from_file #https://github.com/gtfierro/xboswave/blob/master/python/pyxbos/pyxbos/process.py
from pyxbos.drivers import pbc  #https://github.com/gtfierro/xboswave/tree/master/python/pyxbos/pyxbos/drivers
# above imports LPBCProcess, SPBCProcess, EnergiseMessage, LPBCStatus, LPBCCommand, SPBC, EnergiseError
import sys
import numpy as np
import pandas as pd
import time as pytime
import warnings
import logging
import requests
from requests_futures.sessions import FuturesSession
from pymodbus.client.sync import ModbusTcpClient as ModbusClient

#Inverter API
from json import dumps, loads
from datetime import datetime, timedelta
from pathlib import *
import csv
from time import sleep, time, gmtime, mktime
# from Development.InverterEXTAPI import Flexgrid_API
# from Development.InverterControl import ModbusRTUClient
# from Development.convert_data import *

warnings.simplefilter(action='ignore', category=FutureWarning)
logging.basicConfig(level="INFO", format='%(asctime)s - %(name)s - %(message)s')

from PIcontroller import *
from LQRcontroller import *

#HHERE there is a Q-offset of +/- 100 or 200 VARs. need to take this into account and cancel it
#address this with internal feedback for Q command (interal PI controller), based on what was actually sent out?

#HHERE the battery P commands have weird step size issues
#a solution would be turnign down the scaling offsetting the measurements by a set ammount that is pre-designed to meet the phasor target

#HHERE check measurements and commands align with below
'''
Flexlab comands and measurements:
PMU measures positive INTO the battery for both P and Q (inverter looks like an inductor for positive Q measurent)
Inverter Pmax limiting is ambigious to direction
Inverter power factor commands are for Q only, defined positive for reactive power into the network, or OUT of the battery (this is the oppposite of how the PMU measures it)

Changes made for flexlab convention:
Did not change enaythign within PQcalc or phasorI_calc
Did switch the sign of self.Icomp_pu, which is fed intot the impedance estimator only
Did not switch the sign of Pact and Qact (which are positive out of network), or Pcmd and Qcmd (which are positive into the network)
Switched signs of Pact and Qact that are fed into check saturation and ICDI (which still communicates to SPBC using postive into network convention)
inverters are positive P out of the netowrk for batt commands (positive P into the network for inverter-limiting commands)
inverters are positive Q into the netowk (because of how PF is calculated)
load_cmd is still postive into the network (for just P)
modbus is positive out of the network (switched internally)
'''

#HHERE put in t

#to use session.get for parallel API commands you have to download futures: pip install --user requests-futures

class lpbcwrapper(pbc.LPBCProcess): #this is related to super(), inherits attributes and behaviors from pbc.LPBCProcess (which is a wrapper for XBOSProcess)
    def __init__(self, cfg, busId, testcase, nphases, act_idxs, actType, plug_to_phase_idx, timesteplength, currentMeasExists, localSratio=1, localVratio=1, ORT_max_kVA = 500):
        super().__init__(cfg)

        # INITIALIZATION
        self.busId = busId
        self.timesteplength = timesteplength

        #HERE put in optional accumulator term for PI controller

        self.controllerType = 'PI' #set controller to 'PI' or 'LQR'

        if self.controllerType == 'PI':
            # controller gains must be list, even if single phase. can use different gains for each phase
            # e.g. if only actuating on 2 phases (B and C) just put gains in order in list: [#gain B, #gain C]
            print('made a PI controller')
            #3.1
# =============================================================================
#             kp_ang = [0.0034]
#             ki_ang = [0.0677]
#             kp_mag = [0.5670]
#             ki_mag = [3.4497]
# =============================================================================
            
            #3.2
            # alph = 0.4
            # beta = 0.75
            # kp_ang = [0.00108*alph,0.0342*alph]
            # ki_ang = [0.0618*alph,0.0677*alph]
            # kp_mag = [0.6901*beta,1.6522*beta]
            # ki_mag = [3.46*beta,3.5004*beta]
            
            #3.3
# =============================================================================
#             alph = 0.45
#             beta = 0.75
#             kp_ang = [0.0034*alph,0.0034*alph,0.0034*alph]
#             ki_ang = [0.0677*alph,0.0677*alph,0.0677*alph]
#             kp_mag = [0.1750*beta,0.3063*beta,0.8331*beta]
#             ki_mag = [3.5004*beta,3.5004*beta,3.5004*beta]
# =============================================================================

            #5.1
# =============================================================================
#             alph = 0.75
#             kp_ang = [0.004*alph]*3
#             ki_ang = [0.0798*alph]*3
#             kp_mag = [0,0,0]
#             ki_mag = [0,0,0]
# =============================================================================

            # 13 unbal T8
            # alph = 0.45
            # beta = 0.35
            # kp_ang = [0.0034*alph,0.0034*alph,0.0034*alph]
            # ki_ang = [0.0677*alph,0.0677*alph,0.0677*alph]
            # kp_mag = [0.1750*beta,0.3063*beta,0.8331*beta]
            # ki_mag = [3.5004*beta,3.5004*beta,3.5004*beta]

            # 8.1 (33NF)
            alph = 0.2
            beta = 6
            kp_ang = [0.001 * alph, 0.001 * alph, 0.001 * alph]
            ki_ang = [0.2 * alph, 0.2 * alph, 0.2 * alph]
            kp_mag = [0.5 * beta, 0.5 * beta, 0.5 * beta]
            ki_mag = [0.9 * beta, 0.9 * beta, 0.9 * beta]

            self.controller = PIcontroller(nphases, kp_ang, ki_ang, kp_mag, ki_mag)
        elif self.controllerType == 'LQR':
            #If jsut LQR controller is used, from here down should come from the creation of each LPBC, and ultimately the toml file
            Zskpath = 'Zsks/Zsks_pu_' + str(testcase) + '/Zsk_bus' + str(busId) + '.csv'
            if testcase == 'manual': #HERE for debugging, assumes 13bal is used
                Zskpath = 'Zsks/Zsks_pu_' + '13bal' + '/Zsk_bus' + str(busId) + '.csv'
            Zsk_df = pd.read_csv(Zskpath, index_col=0) #index_col=0 bc of how Im saving the df (should have done index = false)
            Zsk_df = Zsk_df.apply(lambda col: col.apply(lambda val: complex(val.strip('()')))) #bc data is complex
            Zskinit = np.asmatrix(Zsk_df.values)
            #LQR controller params
            Qcost = np.eye(nphases*4) #state costs (errors then entegrated errors)
            Rcost = np.eye(nphases*2)*10e-1 #controll costs (P and Q)
            lpAlpha = .1 #DOBC parameter, larger alpha changes estimate faster
            lam = .99 #Zskest parameter, smaller lam changes estimate faster
            use_Zsk_est = 0
            self.controller = LQRcontroller(nphases,timesteplength,Qcost,Rcost,Zskinit,use_Zsk_est,currentMeasExists,lpAlpha,lam)
        else:
            error('error in controller type')

        self.ametek_phase_shift = 0 #in degrees
        self.actType = actType

        self.nphases = nphases
        self.iteration_counter = 0

        self.Pcmd_kVA = np.zeros(nphases)
        self.Qcmd_kVA = np.zeros(nphases) #Pcmd comes from the feedback controller
        self.Pcmd_pu = np.zeros(nphases)  #both Pcmd and Pact are in the local power setting (not the amplified OpalRT setting which is just multiplied by localSratio)
        self.Qcmd_pu = np.zeros(nphases)

        self.Pact_kVA = np.zeros(nphases) #Pactual (measured from pmus, used to calc saturation)
        self.Qact_kVA = np.zeros(nphases)
        self.Pact = np.zeros(nphases)
        self.Qact = np.zeros(nphases)
        self.Pact_pu = np.zeros(nphases)
        self.Qact_pu = np.zeros(nphases)

        self.Vang = np.asarray([np.NaN]*nphases) #all angles should be in radians
        self.Vmag = np.zeros(nphases)
        self.Vmag_pu = np.zeros(nphases)
        self.Vmag_relative = np.zeros(nphases)
        self.Vmag_relative_pu = np.zeros(nphases)
        self.phasor_error_ang = np.zeros(nphases)
        self.phasor_error_mag_pu = np.zeros(nphases)
        self.VmagRef = np.zeros(nphases)
        self.VmagRef_pu = np.zeros(nphases)
        self.VangRef = np.zeros(nphases)

        #Just need to decide what to call unintialized values (probably np.zero if more than 1 dimension)
        #Targets received from SPBC, right now VmagTarg as relative not abosolute
        self.VangTarg = 'initialize' #intialized the first time a phasor_target packet comes from the SPBC, control loop isnt run until a packet is received
        self.VmagTarg = 'initialize' #all angles should be in radians
        # self.VmagTarg_pu = np.zeros(nphases) #rn SPBC sends targets in relative_pu, so these aren't needed
        # self.VmagTarg_relative = np.zeros(nphases)
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
        self.Icomp_pu = np.zeros(nphases,dtype=np.complex_)

        #saturation variables
        self.sat_arrayP = np.ones(nphases) #logic vectors which are 1 if a given phase is not saturated, and zero if it is
        self.sat_arrayQ = np.ones(nphases) #if no current measurements, then these will just stay zero and saturated == 0
        self.Pmax_pu = np.asarray([np.NaN] * nphases) #this signal is used by the SPBC if ICDI is true, otherwise its a nan
        self.Qmax_pu = np.asarray([np.NaN] * nphases)
        self.saturationCounterLimit = 10
        self.Psat = np.ones((nphases, self.saturationCounterLimit)) #set of sat_arrayPs
        self.Qsat = np.ones((nphases, self.saturationCounterLimit))
        self.ICDI_sigP = np.zeros((nphases, 1), dtype=bool) #I Cant Do It signal, defaulted to zero (that it can do it)
        self.ICDI_sigQ = np.zeros((nphases, 1), dtype=bool)

        #phasor calc
        self.local_time_index = [np.NaN]*nphases
        self.ref_time_index = [np.NaN]*nphases

        self.nPhasorReadings = 100
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
        self.loadrack_manuallimit = 1500.
        self.batt_max = 3300.
        self.inv_s_max = 7600. * 0.90  # 0.97 comes from the fact that we are limiting our inverter max to 97% of its true max to prevent issues with running inverter at full power
        self.inv_s_max_commands = 8350.
        self.mode = 4 #How we control inverters mode 1: PV as disturbance, mode 2: PV calculated, mode 3: PV only
        self.batt_cmd = np.zeros(nphases) #battery commands are given in watts
        self.invPperc_ctrl = np.zeros(nphases) #inverter P commnads are given as a percentage of inv_s_max
        self.load_cmd = np.zeros(nphases) #load commands are given in watts
        self.P_PV = np.zeros(nphases)
        self.pf_ctrl = np.ones(nphases)
        # self.flexgrid = Flexgrid_API(inv_ids=[1, 2, 3], portNames=['COM3'], baudrate=115200, parallel=False, safety=True,
        #                         debug=False, ComClient=ModbusRTUClient)
        self.inv_Pmax = 7000 #check with Maxime
        self.inv_Qmax = 5000 #check with Maxime
        self.offset_mode = 2

        IP = '131.243.41.14'
        PORT = 504
        self.client = ModbusClient(IP, port=PORT)

        self.scaling33NF = 3.

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
            print(phasor_target['phasor_targets'])
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
        Vmeas_all_phases = 1
        # Extract latest nPhasorReadings readings from local and ref uPMUs, and put local in phase-order (ref is assumed to be in phase-order)
        for plug in range(nphases): #this will just read the voltage measurements cause its nphases long, even if local_phasors also has current measurements
            if len(local_phasors[plug]) > self.nPhasorReadings:
                dataWindowLength = self.nPhasorReadings
            else:
                dataWindowLength = len(local_phasors[plug])
            phase_idx = plug_to_V_idx[plug]
            ordered_local[phase_idx] = local_phasors[plug][-dataWindowLength:] #this orders local in A,B,C phase order (ref is assumed ot be in A,B,C order)
            ref[plug] = reference_phasors[plug][-dataWindowLength:] #from dataWindowLength back to present, puts Lx2 entries in each entry of local, x2 is for magnitude and phase
            #small chance theres a problem here w copying a mutable data type and not using .copy()

        #this was creating issues when intitial phasor reading wasnt correct
        # if self.Vang == 'initialize':
        #     self.Vang = np.zeros(nphases)
        #     # loops through every phase with actuation
        #     for phase in range(nphases): #phases descrived by a,b,c ordering, but not necessarily a,b,c, all angles are base zero (ie not base -2pi/3 for phase B) bec they are relative angles
        #         # Initialize: Extract measurements from most recent timestamps only for first iteration
        #         V_mag_local = ordered_local[phase][-1]['magnitude']
        #         V_ang_local = ordered_local[phase][-1]['angle'] - self.ametek_phase_shift
        #         V_mag_ref = ref[phase][-1]['magnitude']
        #         V_ang_ref = ref[phase][-1]['angle']
        #         self.Vang[phase] = np.radians(V_ang_local - V_ang_ref)
        #         self.Vmag[phase] = V_mag_local
        #         self.VmagRef[phase] = V_mag_ref
        #         self.Vmag_relative[phase] = V_mag_local - V_mag_ref

        # loops through each set of voltage measurements for each phase
        local_time_index = [np.NaN]*nphases
        ref_time_index = [np.NaN]*nphases
        for phase in range(nphases):
            # loops through every ordered_local uPMU reading starting from most recent
            for local_packet in reversed(ordered_local[phase]):
                # extract most recent ordered_local uPMU reading
                local_time = int(local_packet['time'])
                # loops though every reference uPMU reading starting from most recent
                for ref_packet in reversed(ref[phase]):
                    ref_time = int(ref_packet['time'])
                    
                    #print(f'ref,local,diff: {ref_time},{local_time},{(ref_time-local_time)/1e6}')

                    # check timestamps of ordered_local and reference uPMU if within 2 ms
                    if abs(ref_time - local_time) <= self.pmuTimeWindow:
                        local_time_index[phase] = ordered_local[phase].index(local_packet)
                        ref_time_index[phase] = ref[phase].index(ref_packet)
                        # Extract measurements from closest timestamps
                        V_mag_local = ordered_local[phase][local_time_index[phase]]['magnitude']
                        V_ang_local = ordered_local[phase][local_time_index[phase]]['angle'] - self.ametek_phase_shift
                        V_mag_ref = ref[phase][ref_time_index[phase]]['magnitude']
                        V_ang_ref = ref[phase][ref_time_index[phase]]['angle']

                        V_mag_local = V_mag_local * self.scaling33NF
                        V_mag_ref = V_mag_ref * self.scaling33NF

                        # calculates relative phasors
                        self.Vang[phase] = np.radians(V_ang_local - V_ang_ref)
                        self.Vmag[phase] = V_mag_local
                        self.VmagRef[phase] = V_mag_ref
                        self.Vmag_relative[phase] = V_mag_local - V_mag_ref
                        flag[phase] = 0
                        break
                if flag[phase] == 0:
                    break
            if flag[phase] == 1:
                print('No timestamp found bus ' + str(self.busId) + ' phase ' + str(phase))
                Vmeas_all_phases = 0
        return (self.Vang,self.Vmag,self.VmagRef,self.Vmag_relative, local_time_index, ref_time_index, dataWindowLength, Vmeas_all_phases) #returns the self. variables bc in case a match isnt found, they're already initialized



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
            for phase in range(nphases): #this is in a,b,c ordering, but not necessarily a,b,c, all angles are base zero (ie not base -2pi/3 for phase B) bec they are relative angles
                # Initialize: Extract measurements from most recent timestamps only for first iteration
                self.Imag[phase] = ordered_local[phase][-1]['magnitude']
                I_ang_local = ordered_local[phase][-1]['angle']
                I_ang_ref = ref[phase][-1]['angle']
                self.Iang[phase] = np.radians(I_ang_local - I_ang_ref)
        for phase in range(nphases):
            if local_time_index[phase] != np.NaN and ref_time_index[phase] != np.NaN:
                # Extract measurements from closest timestamps
                self.Imag[phase] = ordered_local[phase][local_time_index[phase]]['magnitude']
                I_ang_local = ordered_local[phase][local_time_index[phase]]['angle']
                I_ang_ref = ref[phase][ref_time_index[phase]]['angle']
                self.Iang[phase] = np.radians(I_ang_local - I_ang_ref)  #uses self. so it defaults to previous value
        return (self.Iang, self.Imag)



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
            theta[phase_idx] = np.radians(V_ang[phase_idx] - I_ang[phase_idx]) #angle comes in in degrees, theta is calced for each phase, so there shouldnt be any 2pi/3 offsets
            # P = (VI)cos(theta), Q = (VI)sin(theta)
            Pact_kVA[phase_idx] = V_mag[phase_idx] * I_mag[phase_idx] * (np.cos(theta[phase_idx]))/1000
            Qact_kVA[phase_idx] = V_mag[phase_idx] * I_mag[phase_idx] * (np.sin(theta[phase_idx]))/1000

        return (Pact_kVA,Qact_kVA)


    def checkSaturationWoImeas(self, nphases, Vcomp, VPcmd_kVA, Qcmd_kVA,):
        # compare self.VcompPrev w Vcomp and if it keeps missing in the same direction declare that its saturated
        #could be confused by Q offset
        pass
        return


    def checkSaturation(self, nphases, Pact, Qact, Pcmd_kVA, Qcmd_kVA, P_PV):
        Pcmd = Pcmd_kVA * 1000
        Qcmd = Qcmd_kVA * 1000
        Pact_VA = Pact*1000
        Qact_VA = Qact*1000
        if self.actType == 'inverter':
            '''
            # find indicies where Pact + tolerance is less than Pcmd
            indexP = np.where(abs(Pact_VA + (0.03 * Pcmd)) < abs(Pcmd))[0] #will be zero if Pcmd is zero
            print(f'PactVA: {Pact_VA}, abs(Pcmd): {abs(Pcmd)}')
            #indexP = np.where(abs(Pact_VA - P_PV) + 500 < abs(Pcmd))[0] #specific to step size of inverters
            #indexP = np.where(abs(Pact_VA) < abs(Pcmd))[0]
            # find indicies where Qact + tolerance is less than Qcmd
            indexQ = np.where(abs(Qact_VA + (0.03 * Qcmd)) < abs(Qcmd))[0]
            print(f'QactVA: {abs(Qact_VA)}, abs(Qcmd): {abs(Qcmd)}')
            #indexQ = np.where(abs(Qact_VA) + 250 < abs(Qcmd))[0]
            #indexQ = np.where(abs(Qact_VA) < abs(Qcmd))[0]
            '''

            indexP = np.where(abs(Pcmd)>= self.ORT_max_VA/self.localSratio)[0]
            indexQ = np.where(abs(Qcmd)>= self.ORT_max_VA/self.localSratio)[0]

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
        sat_arrayP = np.ones(nphases) #
        # stop integrator for saturated phases
        for i in indexP:
            sat_arrayP[i] = 0 #0 where saturated
        "Checking for Q saturation (anti-windup control)"
        # initialize saturation counter for each phase
        sat_arrayQ = np.ones(nphases)
        # stop integrator for saturated phases
        for i in indexQ:
            sat_arrayQ[i] = 0
        return(sat_arrayP,sat_arrayQ)


    def determineICDI(self, nphases, sat_arrayP, sat_arrayQ, Pact_pu, Qact_pu):
        # saturation counter check to determine if I Cant Do It signal should be sent to SPBC
        self.Psat = np.append(self.Psat, np.expand_dims(sat_arrayP, axis=1), axis=1)
        self.Psat = self.Psat[:, 1:] #iterates the Psat counter array to include the new value, discards the old
        for phase in range(nphases):
            if phase in np.where(~self.Psat.any(axis=1))[0]: #if each row doesnt have a 1 in it, then send ICDI for that phase
                self.ICDI_sigP[phase] = True
                if self.actType == 'inverter':

                    #self.Pmax_pu[phase] = Pact_pu[phase]
                    self.Pmax_pu[phase] = self.ORT_max_VA /(self.localkVAbase[phase] *1000)
                elif self.actType == 'load':
                    self.Pmax_pu[phase] = (self.loadrackPlimit/2)/(self.localkVAbase[phase]  *1000) #Sratio double counted in localkVAbase
                elif self.actType == 'modbus':
                    self.Pmax_pu[phase] = self.ORT_max_VA /(self.localkVAbase[phase] *1000)
            else:
                self.ICDI_sigP[phase] = False
                self.Pmax_pu[phase] = np.NaN
        self.Qsat = np.append(self.Qsat, np.expand_dims(sat_arrayQ, axis=1), axis=1)
        self.Qsat = self.Qsat[:, 1:]
        for phase in range(nphases):
            if phase in np.where(~self.Qsat.any(axis=1))[0]:
                self.ICDI_sigQ[phase] = True
                if self.actType == 'inverter':

                    #self.Qmax_pu[phase] = Qact_pu[phase]
                    self.Qmax_pu[phase] = self.ORT_max_VA /(self.localkVAbase[phase] *1000)
                elif self.actType == 'load':
                    self.Qmax_pu[phase] = 0
                elif self.actType == 'modbus':
                    self.Qmax_pu[phase] = self.ORT_max_VA /(self.localkVAbase[phase] *1000)
            else:
                self.ICDI_sigQ[phase] = False
                self.Qmax_pu[phase] = np.NaN
        return (self.ICDI_sigP, self.ICDI_sigQ, self.Pmax_pu, self.Qmax_pu)


    def httptoInverters(self, nphases, act_idxs, Pcmd_kVA, Qcmd_kVA, Pact, inv_Pmax, inv_Qmax): #, local_P_limit, local_Q_limit):
        # hostname: http://131.243.41.48:
        # port: 9090
        #  Sends P and Q command to actuator
        #needs an up-to-date Pact, which requires a current measurement
        #HERE Pact is defined as positive out of the network into the inverter (Pact, Pbatt and P_PV are all positive out of the network in flexlab). This convention should be swithced in later implemetations, but shouldnt require changing (too many) signs
        Pcmd_VA = Pcmd_kVA*1000 # *** SIGNS CHANGED 5/21/20!!! ***
        Qcmd_VA = Qcmd_kVA*1000 #HERE Power factor as positive for Q into the network, which is backwards of the rest of the conventions
        #initialize parallel API command:
        session = FuturesSession()
        urls = []
        commandReceipt = np.zeros(nphases).tolist()
        if self.mode == 1: #1: PV as disturbance
            self.P_PV = (Pact*1000) - self.batt_cmd #P_PV is defined as positive into the solar panel (to be consistent w battery convention) #batt_cmd from last round, still in effect
            for i, inv in zip(range(nphases), act_idxs):
                self.batt_cmd[i] = int(round(Pcmd_VA[i])) #in mode 1 the battery is controlled directly
                if abs(self.batt_cmd[i]) > self.batt_max:
                    self.batt_cmd[i] = int(np.sign(Pcmd_VA[i]) * self.batt_max)
                if ((self.batt_cmd[i] + self.P_PV[i])**2 + Qcmd_VA[i]**2) > (self.inv_s_max)**2: #if Qcmd is over the max, set it to the max for the given P command (favors P over Q)
                    Qcmd_VA[i] = np.sign(Qcmd_VA[i]) * np.sqrt((self.inv_s_max)**2 - (self.batt_cmd[i] + self.P_PV[i])**2) #what happens by default? it probably maintains the PF command and just produces less P (and the battery curtails itself naturally)
                self.pf_ctrl[i] = (np.sign(Qcmd_VA[i])*-1. * abs(self.batt_cmd[i] + self.P_PV[i])) / \
                          (np.sqrt(((self.batt_cmd[i] + self.P_PV[i])**2) + (Qcmd_VA[i]**2))) #self.batt_cmd[i] + P_PV is ~ the full P flowing through the inverter
                if np.abs(self.pf_ctrl[i]) < 0.1:
                    pf_ctrl = 0.1
                print(f'pf cmd: {self.pf_ctrl[i]}, batt cmd: {self.batt_cmd[i]}')
                urls.append(f"http://131.243.41.47:9090/control?Batt_ctrl={self.batt_cmd[i]},pf_ctrl={self.pf_ctrl[i]},inv_id={inv}")
        if self.mode == 2: #mode 2: PV calculated
            self.P_PV = Pact - self.batt_cmd #batt_cmd from last round, still in effect
            for i, inv in zip(range(nphases), act_idxs):
                self.batt_cmd[i] = int(round(Pcmd_VA[i] - self.P_PV[i])) #in mode 2 the battery and PV are controlled jointly
                if abs(self.batt_cmd[i]) > self.batt_max:
                    self.batt_cmd[i] = int(np.sign(Pcmd_VA[i]) * self.batt_max)
                if (self.batt_cmd[i]**2 + Qcmd_VA[i]**2) > (self.inv_s_max)**2: #if Qcmd is over the max, set it to the max for the given P command (favors P over Q)
                    Qcmd_VA[i] = np.sign(Qcmd_VA[i]) * np.sqrt((self.inv_s_max)**2 - self.batt_cmd[i]**2)
                self.pf_ctrl[i] = (np.sign(Qcmd_VA[i])*-1. * abs(self.batt_cmd[i])) / \
                          (np.sqrt((self.batt_cmd[i]**2) + (Qcmd_VA[i]**2))) #self.batt_cmd is ~ the full P flowing through the inverter
                if np.abs(self.pf_ctrl[i]) < 0.1:
                    pf_ctrl = 0.1
                print(f'pf cmd: {self.pf_ctrl[i]}, batt cmd: {self.batt_cmd[i]}')
                urls.append(f"http://131.243.41.47:9090/control?Batt_ctrl={self.batt_cmd[i]},pf_ctrl={self.pf_ctrl[i]},inv_id={inv}")
        if self.mode == 3: #mode 3: PV only
            Pcmd_VA = -Pcmd_VA #HERE for inverter control, P is postive into the network (offsets negative at the beginning of this function)
            for i, inv in zip(range(nphases), act_idxs): #HERE make sure act_idxs is working
                Inv_Pperc_max = 97
                #in mode 3 p_ctrl is used instead of battery control, to control PV
                if Pcmd_VA[i] < 0:
                    Pcmd_VA[i] = 0
                self.invPperc_ctrl[i] = (Pcmd_VA[i] / self.inv_s_max_commands) * 100 #invPperc_ctrl cannot be negative
                if self.invPperc_ctrl[i] > Inv_Pperc_max:
                    self.invPperc_ctrl[i] = Inv_Pperc_max
                    self.pf_ctrl[i] = 1
                    # pf_ctrl = ((np.sign(Qcmd_VA[i]) * -1.0) * Inv_Pperc_max
                else:
                    self.pf_ctrl[i] = (np.sign(Qcmd_VA[i])*-1. * abs(Pcmd_VA[i])) / \
                              (np.sqrt((Pcmd_VA[i] ** 2) + (Qcmd_VA[i] ** 2)))
                if np.abs(self.pf_ctrl[i]) < 0.1:
                    self.pf_ctrl[i] = 0.1
                print(f'pf cmd: {self.pf_ctrl[i]}, batt cmd: {self.batt_cmd[i]}')
                urls.append(f"http://131.243.41.47:9090/control?P_ctrl={self.invPperc_ctrl[i]},pf_ctrl={self.pf_ctrl[i]},inv_id={inv}")


        if self.mode == 4: #mode 4: HIL2 dynamic P and Q control
            print(f'PCMD_VA: {Pcmd_VA}')
            print(f'QCMD_VA: {Qcmd_VA}')
            Pcmd_VA = abs(
                Pcmd_kVA * 1000)  # abs values for working only in quadrant 1. Will use modbus to determine quadrant
            Qcmd_VA = abs(
                Qcmd_kVA * 1000)  # abs values for working only in quadrant 1. Will use modbus to determine quadrant

            # CIL OFFSET FUNCATIONALITAY (to reduce scaling --> smaller oscillation from Q control)
            if self.offset_mode == 1:
                id = 3
                offset_inc = 100
                CIL_offset_max = self.ORT_max_VA/1000 - offset_inc
                Pcmd_ORT_VA = Pcmd_VA * self.localSratio
                Qcmd_ORT_VA = Qcmd_VA * self.localSratio
                P_offset_inc_idx = Pcmd_ORT_VA // (offset_inc*1000)
                Q_offset_inc_idx = Qcmd_ORT_VA // (offset_inc*1000)
                CIL_offset = offset_inc * np.concatenate([P_offset_inc_idx,Q_offset_inc_idx]) # this is a value that gets sent as kW/kVar direct to ORT via modbus
                mtx = [0] * nphases*2
                # cap at max offset
                for i in range(nphases*2):
                    if CIL_offset[i] > CIL_offset_max:
                        CIL_offset[i] = CIL_offset_max
                        if i < 3:
                            print(f'P_CIL_offset[{i}] over max - reduced to {CIL_offset_max}')
                        if i >= 3:
                            print(f'Q_CIL_offset[{i-3}] over max - reduced to {CIL_offset_max}')
                # send as P1,Q1,P2,Q2,P3,Q3 to 301 - 306
                mtx[0:nphases*2-1:2] = CIL_offset[0:nphases]
                mtx[1:nphases*2:2] = CIL_offset[nphases:nphases*2]
                mtx_register = np.arange(301,306+1).tolist()
                # update inverter command to account for CIL offset
                offset_steps = self.ORT_max_VA/1000/offset_inc
                offsetSratio = self.localSratio/offset_steps
                Pcmd_ORT_VA_rem = Pcmd_ORT_VA - P_offset_inc_idx * offset_inc * 1000
                Qcmd_ORT_VA_rem = Qcmd_ORT_VA - Q_offset_inc_idx * offset_inc * 1000
                if self.offset_mode == 1:
                    Pcmd_VA = Pcmd_ORT_VA_rem/offsetSratio
                    Qcmd_VA = Qcmd_ORT_VA_rem/offsetSratio
                print('OFFSET COMMANDS:')
                print(f'Pcmd_ORT_VA: {Pcmd_ORT_VA}')
                print(f'Pcmd_ORT_VA_rem: {Pcmd_ORT_VA_rem}')
                print(f'mtx: {mtx}')
                print(f'Pcmd_rem: {Pcmd_VA}')
                print(f'Qcmd_rem: {Qcmd_VA}')
            if self.offset_mode == 2:
                id = 3
                offset_inc = 100
                offset_steps = self.ORT_max_VA/1000/offset_inc
                offsetSratio = self.localSratio/offset_steps

                inv_offset_perc = offset_inc/(self.ORT_max_VA/1000)
                CIL_offset_perc = 1 - inv_offset_perc
                CIL_offset_max = self.ORT_max_VA/1000 - offset_inc
                Pcmd_ORT_VA = Pcmd_VA * self.localSratio
                Qcmd_ORT_VA = Qcmd_VA * self.localSratio
                CIL_offset = CIL_offset_perc * np.concatenate([Pcmd_ORT_VA,Qcmd_ORT_VA]) / 1000 # this is a value that gets sent as kW/kVar direct to ORT via modbus
                mtx = [0] * nphases*2
                # cap at max offset
                for i in range(nphases*2):
                    if CIL_offset[i] > CIL_offset_max:
                        CIL_offset[i] = CIL_offset_max
                        if i < 3:
                            print(f'P_CIL_offset[{i}] over max - reduced to {CIL_offset_max}')
                        if i >= 3:
                            print(f'Q_CIL_offset[{i-3}] over max - reduced to {CIL_offset_max}')
                # send as P1,Q1,P2,Q2,P3,Q3 to 301 - 306
                mtx[0:nphases*2-1:2] = CIL_offset[0:nphases]
                mtx[1:nphases*2:2] = CIL_offset[nphases:nphases*2]
                mtx_register = np.arange(301,306+1).tolist()

                Pcmd_VA = Pcmd_ORT_VA * inv_offset_perc / offsetSratio
                Qcmd_VA = Qcmd_ORT_VA * inv_offset_perc / offsetSratio
                print('OFFSET COMMANDS:')
                print(f'Pcmd_ORT_VA: {Pcmd_ORT_VA}')
                print(f'Pcmd_ORT_VA_inv: {Pcmd_ORT_VA * inv_offset_perc}')
                print(f'mtx: {mtx}')
                print(f'Pcmd_inv: {Pcmd_VA}')
                print(f'Qcmd_inv: {Qcmd_VA}')
            for i in range(len(Pcmd_VA)):
                if Pcmd_VA[i] > self.ORT_max_VA/self.localSratio:
                    Pcmd_VA[i] = self.ORT_max_VA/self.localSratio
                    print(i,' inverter: P over ORT MAX ([0,1,2] -> [1,2,3])')
                if Qcmd_VA[i] > self.ORT_max_VA/self.localSratio:
                    Qcmd_VA[i] = self.ORT_max_VA/self.localSratio
                    print(i,' inverter: Q over ORT MAX ([0,1,2] -> [1,2,3])')
            print(f'absolute value of P/Q:{Pcmd_VA},{Qcmd_VA}')

            # +50 to Q is a constant offset due to hardware measurement error
            # +1000 to P is a constant offset that then gets subtracted out in an effort to reduce the change in pf across the range of actuation values
            # i.e. 1000 - 2000 output by the inverter is actually 0 - 1000 in the model.
            Pcmd_perc = (Pcmd_VA + 1000) / inv_Pmax * 100  # Pcmd to inverters must be a percentage of Pmax
            Qcmd_perc = (Qcmd_VA + 50) / inv_Qmax * 100 # Qcmd to inverters must be a percentage of Qmax

            act_idxs = act_idxs.tolist()
            for i in range(len(Pcmd_perc)):  # checks Pcmd for inverter limit
                if Pcmd_perc[i] > 50:
                    Pcmd_perc[i] = 50
                if Pcmd_perc[i] < 0.1:
                    Pcmd_perc[i] = 0.1
            for j in range(len(Qcmd_perc)):  # checks Qcmd for inverter limit
                if Qcmd_perc[j] > 50:
                    Qcmd_perc[j] = 50
                if Qcmd_perc[j] < 0.1:
                    Qcmd_perc[j] = 0.1
            print(f'Pcmd_perc: {Pcmd_perc}')
            print(f'Qcmd_perc: {Qcmd_perc}')
            # Debugging section
            # if 3 or 2 in act_idxs:
            #     print('warning phase B or C activated')
            #     return
            # for Pcmd_perc_phase, inv in zip(Pcmd_perc, act_idxs):
            #     Pcmd_perc_phase = Pcmd_perc_phase.item()  # changes data type from numpy to python int/float
            #     inv = inv.item()  # changes data type
            #     if inv == 1:
            #         urls.append(f"http://131.243.41.48:9090/control?dyn_P_ctrl={Pcmd_perc_phase},inv_id={inv}")
            #     else:
            #         print('inv != 1')
            #         return

            for Pcmd_perc_phase, Qcmd_perc_phase, inv in zip(Pcmd_perc, Qcmd_perc, act_idxs):
                Pcmd_perc_phase = Pcmd_perc_phase.item()  # changes data type from numpy to python int/float
                Qcmd_perc_phase = Qcmd_perc_phase.item()  # changes data type
                if type(inv) != int:
                    inv = inv.item()  # changes data type
                urls.append(f"http://131.243.41.48:9090/control?dyn_P_ctrl={Pcmd_perc_phase},dyn_Q_ctrl={Qcmd_perc_phase},inv_id={inv}")

        responses = map(session.get, urls)
        results = [resp.result() for resp in responses]
        for i in range(nphases):
            if results[i].status_code == 200:
                commandReceipt[i] = 'success'
            else:
                commandReceipt[i] = 'failure'
        print(f'INV COMMAND RECEIPT: {commandReceipt}')
        if self.offset_mode == 1 or self.offset_mode == 2:
            try:
                self.client.connect()
                for i in range(len(mtx)):
                    self.client.write_registers(int(mtx_register[i]), int(mtx[i]), unit=id)
                print(f'sent offsets: {mtx}')        
            except Exception as e:
                print(e)        
            finally:
                self.client.close()
        return commandReceipt

    def API_inverters(self, act_idxs, Pcmd_kVA, Qcmd_kVA, inv_Pmax, inv_Qmax, flexgrid):
        Pcmd_VA = abs(Pcmd_kVA*1000) #abs values for working only in quadrant 1. Will use modbus to determine quadrant
        Qcmd_VA = abs(Qcmd_kVA*1000) #abs values for working only in quadrant 1. Will use modbus to determine quadrant
        Pcmd_perc = Pcmd_VA/inv_Pmax #Pcmd to inverters must be a percentage of Pmax
        Qcmd_perc = Qcmd_VA/inv_Qmax #Qcmd to inverters must be a percentage of Qmax
        act_idxs = act_idxs.tolist()
        for i in range(len(Pcmd_perc)): # checks Pcmd for inverter limit
            if Pcmd_perc[i] > 100:
                Pcmd_perc[i] = 100
        for j in range(len(Qcmd_perc)): # checks Qcmd for inverter limit
            if Qcmd_perc[j] > 100:
                Qcmd_perc[j] = 100
        for Pcmd_perc_phase, inv in zip(Pcmd_perc, act_idxs):
            Pcmd_perc_phase = Pcmd_perc_phase.item() #changes data type from numpy to python int/float
            inv = inv.item() #changes data type
            flexgrid.set_dyn_P(Pcmd_perc_phase,inv)
        for Qcmd_perc_phase, inv in zip(Qcmd_perc, act_idxs):
            Qcmd_perc_phase = Qcmd_perc_phase.item() #changes data type
            inv = inv.item() #changes data type
            flexgrid.set_dyn_Q(Qcmd_perc_phase,inv)


    def httptoLoads(self, nphases, act_idxs, Pcmd_kVA, Qcmd_kVA):
        #load commands are between 0 and 2000, but from the LPBC's perspective it can control between -1000 and 1000 W, with 1000 W collocated
        #Pcmd is power into the network, but load commands are just load power (power of out of the network)
        Pcmd_VA = Pcmd_kVA*1000
        Qcmd_VA = Qcmd_kVA*1000
        #initialize parallel API command:
        session = FuturesSession()
        urls = []
        commandReceipt = np.zeros(nphases).tolist()
        for i, group in zip(range(nphases), act_idxs): #same as enumerate
            self.load_cmd[i] = int(np.round((-1. * Pcmd_VA[i]) + self.loadrackPlimit/2)) # -1* bc command goes to a load not an inverter, +self.loadrackPlimit/2 centers the command around 0
            if self.load_cmd[i] > self.loadrack_manuallimit: #self.loadrackPlimit:
                urls.append(f"http://131.243.41.59:9090/control?group_id={group},P_ctrl={self.loadrack_manuallimit}")
            elif self.load_cmd[i] < 0:
                urls.append(f"http://131.243.41.59:9090/control?group_id={group},P_ctrl=0")
            else:
                urls.append(f"http://131.243.41.59:9090/control?group_id={group},P_ctrl={self.load_cmd[i]}")
        responses = map(session.get, urls)
        results = [resp.result() for resp in responses]
        for i in range(nphases):
            if results[i].status_code == 200:
                commandReceipt[i] = 'success'
            else:
                commandReceipt[i] = 'failure'
        return commandReceipt

    def modbustoOpal_quadrant(self, Pcmd_kVA, Qcmd_kVA, Pact, Qact, act_idxs, client):
        id = 3
        inv_1 = 101
        inv_2 = 102
        inv_3 = 103
    
        # value mapping - 1: [-1, -1], 2: [1, -1], 3: [-1, 1], 4: [1, 1]
        # multipliers to inverter values [P, Q] - positive inverter values corresponds to injecting P and Q (value 4)
        # FLEXLAB'S QUADRANT CONVENTION 5/22/20 Flexlab set up quadrant convention and will take care of rest into ephasorsim
        # Quadrant 1: P consume, Q consume
        # Quadrant 2: P inject, Q consume
        # Quadrant 3: P consume, Q inject
        # Quadrant 4: P inject, Q inject
        # old ^
        # new (6/1/20)
        # 4: +P, -Q (for model: P inj, Q cons)
        # 3: -P, -Q (for model: P cons, Q cons)
        # 2: +P, +Q (for model: P inj, Q inj)
        # 1: -P, +Q (for model: P cons, Q inj)

        inv_act_idxs_registers = [inv_1,inv_2,inv_3]
        value = [0] * len(act_idxs)
        for i in range(len(act_idxs)):
            if Pcmd_kVA[i] >= 0 and Qcmd_kVA[i] >= 0:  # quadrant 1
                value[i] = 2
            if Pcmd_kVA[i] < 0 and Qcmd_kVA[i] >= 0:  # quadrant 2
                value[i] = 1
            if Pcmd_kVA[i] < 0 and Qcmd_kVA[i] < 0:  # quadrant 3
                value[i] = 3
            if Pcmd_kVA[i] >= 0 and Qcmd_kVA[i] < 0:  # quadrant 4
                value[i] = 4
        print(f'registers 2: {inv_act_idxs_registers}')
        print(f'values 2: {value}')
        try:
            client.connect()
            for i in range(len(act_idxs)):  # write quadrant changes to modbus registers
                client.write_registers(inv_act_idxs_registers[i], value[i], unit=id)
                print('Quadrant for inv:', inv_act_idxs_registers[i], 'to quadrant', value[i])
        except Exception as e:
            print(e)
        finally:
            client.close()
        return

    def modbustoOpal(self, nphases, Pcmd_kVA, Qcmd_kVA, ORT_max_VA, localSratio, client ):
        Pcmd_VA = -1 * (Pcmd_kVA * 1000) #sign negation is convention of modbus
        Qcmd_VA = -1 * (Qcmd_kVA * 1000) #sign negation is convention of modbus
        id = 3
        # Connect to client
        client.connect()
        for phase in range(nphases):
            print('Opal Pcmd_VA[phase] : ' + str(Pcmd_VA[phase]))
            print('Opal Qcmd_VA[phase] : ' + str(Qcmd_VA[phase]))
            print('ORT_max_VA/localSratio : ' + str(ORT_max_VA/localSratio))
            if abs(Pcmd_VA[phase]) > ORT_max_VA/localSratio:
                print('Pcmd over Opal limit')
                Pcmd_VA[phase] = np.sign(Pcmd_VA[phase]) * ORT_max_VA/localSratio
            if abs(Qcmd_VA[phase]) > ORT_max_VA/localSratio:
                print('Qcmd over Opal limit')
                Qcmd_VA[phase] = np.sign(Qcmd_VA[phase]) * ORT_max_VA/localSratio

        # P,Q commands in W and VAR (not kilo)

        if nphases == 3:
            P1, P2, P3 = abs(Pcmd_VA[0]), abs(Pcmd_VA[1]), abs(Pcmd_VA[2])
            Q1, Q2, Q3 = abs(Qcmd_VA[0]), abs(Qcmd_VA[1]), abs(Qcmd_VA[2])
        # TODO modbus only: manually change phase actuation on modbus here if needed on different phase
        elif nphases == 1:
            P1, P2, P3 = abs(Pcmd_VA[0]), 0, 0
            Q1, Q2, Q3 = abs(Qcmd_VA[0]), 0, 0

        # set signs of commands through sign_vec
        #           P,Q      1 is positive, 0 is negative
        sign_vec = []
        for p, q in zip(Pcmd_VA, Qcmd_VA):
            if p >= 0:
                sign_vec.append(1)
            if p < 0:
                sign_vec.append(0)
            if q >= 0:
                sign_vec.append(1)
            if q < 0:
                sign_vec.append(0)
        if nphases == 3:
            sign_base = 2 ** 5 * sign_vec[0] + 2 ** 4 * sign_vec[1] + 2 ** 3 * sign_vec[2] + 2 ** 2 * sign_vec[
            3] + 2 ** 1 * sign_vec[4] + 2 ** 0 * sign_vec[5]
        # TODO modbus only: manually change phase actuation on modbus here for sign base if needed on different phase
        elif nphases == 1:
            sign_base = 2 ** 5 * sign_vec[0] + 2 ** 4 * sign_vec[1]

        mtx = [P1, Q1, P2, Q2, P3, Q3, sign_base]
        print('mtx : ' + str(mtx))
        mtx_register = np.arange(1, 8).tolist()
        try:
            # write switch positions for config
            for i in range(len(mtx)):
                client.write_registers(int(mtx_register[i]), int(mtx[i]), unit=id)
            result = 'sent'
        except Exception as e:
            result = ('exceptions', e)
        finally:
            client.close()
        return result


    def initializeActuators(self, mode):
        if mode == 0:
            return
        elif mode == 1 or mode == 2:
            responseInverters = requests.get("http://131.243.41.47:9090/control?P_ctrl=97,Batt_ctrl=0")
        elif mode == 3:
            responseInverters = requests.get("http://131.243.41.47:9090/control?P_ctrl=0,Batt_ctrl=0")
        #responseLoads = requests.get(f"http://131.243.41.118:9090/control?P_ctrl=0")
        if responseInverters.status_code != 200 or responseLoads.status_code != 200:
            error('Error with actuator initialization, responseInverters.status_code = ' + str(responseInverters.status_code) + 'responseLoads.status_code = ' + str(responseLoads.status_code))
        return (responseInverters.status_code, responseLoads.status_code)


    def statusforSPBC(self, phases, phasor_error_mag_pu, phasor_error_ang, ICDI_sigP, ICDI_sigQ, Pmax_pu, Qmax_pu):
        status = {}
        # status's keys should be lists
        status['phases'] = phases
        status['phasor_errors'] = {
                'V': list(phasor_error_mag_pu.ravel()), #ravel flatens the dimensions
                'delta': list(phasor_error_ang.ravel())
            }
        status['p_saturated'] = list(ICDI_sigP.ravel())
        status['q_saturated'] = list(ICDI_sigQ.ravel())
        status['p_max'] = list(Pmax_pu.ravel())
        status['q_max'] = list(Qmax_pu.ravel())
        return(status)

    def PhasorV_ang_wraparound(self, Vang_relative, nphases):
        # accounts for special cases where one angle crosses zero while the other is behind zero
        Vang_relative_wrap = Vang_relative
        for phase in range(nphases):
            if abs(Vang_relative[phase]) > np.radians(300.):
                if Vang_relative[phase] > 0:
                    Vang_relative_wrap[phase] = Vang_relative[phase] - np.radians(360.)
                elif Vang_relative[phase] < 0:
                    Vang_relative_wrap[phase] = Vang_relative[phase] + np.radians(360.)
        return Vang_relative_wrap

    def save_actuation_data(self, phases, P_cmd, Q_cmd, P_act, Q_act, P_PV, Batt_cmd, pf_ctrl):
        log_actuation= {}
        
        log_actuation['phases'] = phases
        log_actuation['P_cmd'] = P_cmd.tolist() 
        log_actuation['Q_cmd'] = Q_cmd.tolist()
        log_actuation['P_act'] = P_act.tolist()
        log_actuation['Q_act'] = Q_act.tolist()
        log_actuation['P_PV'] = (P_PV/1000).tolist()
        log_actuation['Batt_cmd'] = (Batt_cmd/1000).tolist()
        log_actuation['pf_ctrl'] = pf_ctrl.tolist()

        return log_actuation

    #step gets called every (rate) seconds starting with init in LPBCProcess within do_trigger/trigger/call_periodic (XBOSProcess) with:
    #status = self.step(local_phasors, reference_phasors, phasor_targets)
    def step(self, local_phasors, reference_phasors, phasor_target): #HERE what happens when no PMU readings are given (Gabe), maybe step wont be called
        iterstart = pytime.time()
        self.iteration_counter += 1
        print('iteration counter bus ' + str(self.busId) + ' : ' + str(self.iteration_counter))
        if self.iteration_counter > 1:
            print(f'time since last iteration {iterstart-self.iterstart}')
        self.iterstart = pytime.time()
        
        #Initilizes actuators, makes sure you're getting through to them
        if self.iteration_counter == 1:
            pass
            #HERE commented out for debugging
            # (responseInverters, responseLoads) = self.initializeActuators(self.mode) #throws an error if initialization fails

        if phasor_target is None and self.VangTarg == 'initialize':
            print('No target received by SPBC bus ' + str(self.busId))
            return #don't need to return a status, when there isnt one to report
        else:
            if phasor_target is None:
                print('No target received by SPBC: Using last received target ' + str(self.busId))
            else:
                #get targets and bases from phasor_target, sent by the SPBC
                #values are ordered as: A,B,C according to availability, using the names given to the targets (by the SPBC)
                #VmagTarg is given as VmagTarg_relative_pu rn from the SPBC
                (self.VmagTarg_relative_pu, self.VangTarg, self.kVbase, self.network_kVAbase, self.status_phases) = self.targetExtraction(phasor_target)
                print('VmagTarg_relative_pu bus ' + str(self.busId) + ' : ' + str(self.VmagTarg_relative_pu))
                print('VangTarg bus ' + str(self.busId) + ' : ' + str(self.VangTarg))
                print('kVbase bus ' + str(self.busId) + ' : ' + str(self.kVbase))
                print('network_kVAbase bus ' + str(self.busId) + ' : ' + str(self.network_kVAbase))
                print('status_phases bus ' + str(self.busId) + ' : ' + str(self.status_phases))
                self.kVbase  = np.asarray(self.kVbase)
                self.network_kVAbase = np.asarray(self.network_kVAbase)
                #phasor_target is (perLPBC) data packet from SPBC that contains channels (will be phases once fixed), V, delta, kvbase and kvabase
                self.localkVbase = self.kVbase/self.localVratio
                self.localkVAbase = self.network_kVAbase/self.localSratio
                self.localIbase = self.localkVAbase/self.localkVbase
                print('self.localSratio : ' + str(self.localSratio))
                print('self.localkVAbase : ' + str(self.localkVAbase))
                print('self.localkVbase : ' + str(self.localkVbase))

            # calculate relative voltage phasor
            #the correct PMUs for voltage and current (ie uPMUP123 and uPMU123) are linked in the configuration phase, so local_phasors are what you want (already)
            #values are ordered as: A,B,C according to availability, using self.plug_to_phase_map
            (self.Vang,self.Vmag,self.VmagRef,self.Vmag_relative, local_time_index, ref_time_index, dataWindowLength, Vmeas_all_phases) = self.phasorV_calc(local_phasors, reference_phasors, self.nphases, self.plug_to_V_idx)
            if any(np.isnan(self.Vang)):
                print('Every phase has not received a relative phasor measurement yet, bus ' + str(self.busId))
                return
            if Vmeas_all_phases == 0:
                print('Didnt receive a measurement for each phase, not acting')
                return
            self.Vang = self.PhasorV_ang_wraparound(self.Vang, self.nphases)
            self.Vmag_pu = self.Vmag / (self.localkVbase * 1000) # absolute
            self.Vmag_relative_pu = self.Vmag_relative / (self.localkVbase * 1000) #this and the VmagTarg_relative_pu line divides Vmag_ref by self.localkVbase which may create an issue bc Vref != 1.0pu, but thats okay
            self.VmagRef_pu = self.VmagRef / (self.localkVbase * 1000)
            self.phasor_error_ang = self.VangTarg - self.Vang
            self.phasor_error_mag_pu = self.VmagTarg_relative_pu - self.Vmag_relative_pu
            self.VmagTarg_pu = self.VmagTarg_relative_pu + self.VmagRef_pu #VmagTarg is given as VmagTarg_relative_pu rn from the SPBC
            print('Vmag_pu bus ' + str(self.busId) + ' : ' + str(self.Vmag_pu))
            print('VmagRef_pu bus ' + str(self.busId) + ' : ' + str(self.VmagRef_pu))
            print('Vmag_relative_pu bus ' + str(self.busId) + ' : ' + str(self.Vmag_relative_pu))
            print('Vang bus ' + str(self.busId) + ' : ' + str(self.Vang))

            #get current measurements, determine saturation if current measurements exist
            if self.currentMeasExists:
                if self.controllerType == 'LQR':
                    (self.Iang,self.Imag) = self.phasorI_calc(local_time_index, ref_time_index, dataWindowLength, local_phasors, reference_phasors, self.nphases, self.plug_to_V_idx) #HERE in Flexlab this positive flowing out of the Network
                    self.Imag_pu = self.Imag / self.localIbase #this takes into account Sratio
                    self.Icomp_pu = self.Imag_pu*np.cos(self.Iang) + self.Imag_pu*np.sin(self.Iang)*1j #Assumed current is positive into the Ametek (postive for positive injection), and Iangs are relative and thus base 0 for all phases
                    self.Icomp_pu = -self.Icomp_pu #HERE, want current to be positive into the network for Z estimation
                (self.Pact, self.Qact) = self.PQ_solver(local_phasors, self.nphases,self.plug_to_V_idx)  #HERE this is positive out of the network, which is backwards of Pcmd and Qcmd, bc thats how PMU 123 is set up in the flexlab # calculate P/Q from actuators
                self.Pact_pu = self.Pact / self.localkVAbase
                self.Qact_pu = self.Qact / self.localkVAbase
            else:
                self.Icomp_pu = [np.NaN]*self.nphases

            #HERE [CHANGED] sign negations on Pact and Qact bc of dicrepancy between Pact convention and Pcmd convention 
            (self.sat_arrayP, self.sat_arrayQ) = self.checkSaturation(self.nphases, self.Pact, self.Qact, self.Pcmd_kVA, self.Qcmd_kVA, self.P_PV)  # returns vectors that are one where unsaturated and zero where saturated, will be unsaturated with initial Pcmd = Qcmd = 0
            (self.ICDI_sigP, self.ICDI_sigQ, self.Pmax_pu, self.Qmax_pu) = self.determineICDI(self.nphases, self.sat_arrayP, self.sat_arrayQ, -self.Pact_pu, -self.Qact_pu) #this and the line above have hardcoded variables for Flexlab tests

            #run control loop
            print('self.phasor_error_mag_pu ' + str(self.phasor_error_mag_pu))
            print('self.phasor_error_ang ' + str(self.phasor_error_ang))
            print('self.sat_arrayP ' + str(self.sat_arrayP))
            print('self.sat_arrayQ ' + str(self.sat_arrayQ))
            if self.controllerType == 'PI':
                (self.Pcmd_pu,self.Qcmd_pu) = self.controller.PIiteration(self.nphases,self.phasor_error_mag_pu, self.phasor_error_ang, self.sat_arrayP, self.sat_arrayQ)
            elif self.controllerType == 'LQR':
                if self.currentMeasExists:
                    (self.Pcmd_pu,self.Qcmd_pu) = self.controller.LQRupdate(self.Vmag_pu, self.Vang, self.VmagTarg_pu, self.VangTarg, self.VmagRef_pu, self.VangRef, self.sat_arrayP, self.sat_arrayQ, self.Icomp_pu) #all Vangs must be in radians
                else:
                    (self.Pcmd_pu,self.Qcmd_pu) = self.controller.LQRupdate(self.Vmag_pu, self.Vang, self.VmagTarg_pu, self.VangTarg, self.VmagRef_pu, self.VangRef, self.sat_arrayP, self.sat_arrayQ)
            print('Pcmd_pu bus ' + str(self.busId) + ' : ' + str(self.Pcmd_pu))
            print('Qcmd_pu bus ' + str(self.busId) + ' : ' + str(self.Qcmd_pu))
            print('localkVAbase bus ' + str(self.busId) + ' : ' + str(self.localkVAbase))

            #HHERE debugging
            # self.Pcmd_pu[1] = 0
            # self.Pcmd_pu[2] = 0
            # self.Qcmd_pu = np.zeros(nphases)

            self.Pcmd_kVA = self.Pcmd_pu * self.localkVAbase #these are positive for power injections, not extractions
            self.Qcmd_kVA = self.Qcmd_pu * self.localkVAbase #localkVAbase takes into account that network_kVAbase is scaled down by localSratio (divides by localSratio)

            if self.actType == 'inverter':
                if self.currentMeasExists or self.mode == 3 or self.mode == 4:
                    self.commandReceipt = self.httptoInverters(self.nphases, self.act_idxs, self.Pcmd_kVA, self.Qcmd_kVA, self.Pact, self.inv_Pmax, self.inv_Qmax) #calculating Pact requires an active current measurement
                    self.modbustoOpal_quadrant(self.Pcmd_kVA, self.Qcmd_kVA, self.Pact, self.Qact, self.act_idxs, self.client)
                    #self.API_inverters(self.act_idxs, self.Pcmd_kVA, self.Qcmd_kVA, self.Pmax, self.Qmax, self.flexgrid)
                    print('inverter command receipt bus ' + str(self.busId) + ' : ' + 'executed')
                else:
                    disp('couldnt send inverter commands because no current measurement available')
            elif self.actType == 'load':
                self.commandReceipt = self.httptoLoads(self.nphases, self.act_idxs, self.Pcmd_kVA, self.Qcmd_kVA)
                print('load command receipt bus ' + str(self.busId) + ' : ' + str(self.commandReceipt))
            elif self.actType == 'modbus':
                result = self.modbustoOpal(self.nphases, self.Pcmd_kVA, self.Qcmd_kVA, self.ORT_max_VA, self.localSratio)
                print('Opal command receipt bus ' + str(self.busId) + ' : ' + str(result))
            else:
                error('actType error')
            
            self.Pact_kVA = self.Pact
            self.Qact_kVA = self.Qact
            
            log_actuation = self.save_actuation_data(self.status_phases, self.Pcmd_kVA, self.Qcmd_kVA, self.Pact_kVA, self.Qact_kVA, self.P_PV, self.batt_cmd, self.pf_ctrl)
            self.log_actuation(log_actuation)
            print(log_actuation)

            status = self.statusforSPBC(self.status_phases, self.phasor_error_mag_pu, self.phasor_error_ang, self.ICDI_sigP, self.ICDI_sigQ, self.Pmax_pu, self.Qmax_pu)
            print(status)
            iterend = pytime.time()
            print(f'~~~ STEP FINISH - iter length: {iterend-iterstart}, epoch: {pytime.time()} ~~~')
            print('')
            if (iterend-iterstart) > rate:
                print('WARNING: LOOP LENGTH LARGER THAN RATE - INCREASE SIZE OF RATE')
                print('')

            return status


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


SPBCname = 'spbc-jasper-1'
# SPBCname = 'spbc-example-jasper'

#Manual entry here to determine test case, phases, etc.
#Test Case
#testcase = '37'
#testcase = '13unb'
#testcase = '13bal'
testcase = 'manual'

acts_to_phase_dict = dict()
actType_dict = dict()
if testcase == '37':
    # subkVAbase = 2500
    pass
elif testcase == '13unb':
    # subkVAbase = 5000
    lpbcidx = ['671','680']
    key = '671'
    acts_to_phase_dict[key] = np.asarray(['A','B','C']) #phase on the network (in simulation)
    actType_dict[key] = 'inverter'
    key = '680'
    acts_to_phase_dict[key] = np.asarray(['','','C']) #the nonzero entries correspond to the actuator indices
    actType_dict[key] = 'load'
elif testcase == '13bal':
    # subkVAbase = 5000
    lpbcidx = ['675'] #may have to set these manually
    for key in lpbcidx: #makes them all three phase inverters
        acts_to_phase_dict[key] = np.asarray(['A','B','C']) #3 phase default #['A','',''] or ['','C',''] or ['A','B','C','A','B','C'] or ['A','','','A','',''] are also examples, ['A','C','B'] and ['B','B','B'] are not allowed (yet)
        actType_dict[key] = 'inverter' #'inverter' or 'load'
#TODO: set test case here
elif testcase == 'manual':
    lpbcidx = ['18'] #nodes of actuation
    key = '18'
    acts_to_phase_dict[key] = np.asarray(['A','B','C']) #which phases to actuate for each lpbcidx # INPUT PHASES
    actType_dict[key] = 'inverter' #choose: 'inverter', 'load', or 'modbus'

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
    elif actType_dict[key] == 'load':
        pmu4_plugs_dict[key] = []
        for i in np.arange(3): #actuator i
            if acts_to_phase_dict[key][i] != '': #if this LPBC uses actuator i
                plug = pmu4_act_to_plug_Map[i] #the pmu plug that corresponds to actuator i
                phase = acts_to_phase_dict[key][i]
                plug_to_phase_dict[key][plug] = phase #places the phase for that wave channel in the plug_to_phase mapping that gets sent to the PMUs
                pmu4_plugs_dict[key].append(plug) #places the PMU measurement corresponding to actuator i on the WAVE channe
        pmu4_plugs_dict[key] = np.asarray(sorted(pmu4_plugs_dict[key])) #orders the PMU channels in 0,1,2 ordering. This is expected by plug_to_phase_dict, which is sorted implicitly
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
pmu123Channels = np.asarray(['uPMU_123/L1','uPMU_123/L2','uPMU_123/L3','uPMU_123/C1','uPMU_123/C2','uPMU_123/C3'])
pmu123PChannels = np.asarray(['uPMU_4/L1','uPMU_4/L2','uPMU_4/L3']) #these also have current channels, but dont need them
pmu4Channels = np.asarray(['uPMU_4/L1','uPMU_4/L2','uPMU_4/L3'])
refChannels = np.asarray(['uPMU_0/L1','uPMU_0/L2','uPMU_0/L3','uPMU_0/C1','uPMU_0/C2','uPMU_0/C3'])

nlpbc = len(lpbcidx)

#cfg file is used to build each LPBC, this is a template that is modified below for each LPBC
cfg_file_template = config_from_file('template.toml') #config_from_file defined in XBOSProcess

#this is HIL specific
inverterScaling = 500/1
loadScaling = 350
CILscaling = 500/3.3

rate = 15

lpbcdict = dict()
for lpbcCounter, key in enumerate(lpbcidx):
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
        cfg['rate'] = rate
        cfg['local_channels'] = list(np.concatenate([pmu123PChannels[pmu123P_plugs_dict[key]], pmu123Channels[3 + pmu123_plugs_dict[key]], pmu123Channels[pmu123_plugs_dict[key]]]))
        #takes voltage measurements from PMU123P, current from PMU123, voltage measurements from PMU123P
        cfg['reference_channels'] = list(refChannels[pmu0_plugs_dict[key]]) #assumes current and voltage plugs are connected the same way
        currentMeasExists = True
        localSratio = inverterScaling
    elif actType == 'load':
        cfg['rate'] = rate
        cfg['local_channels'] = list(pmu4Channels[pmu4_plugs_dict[key]])
        cfg['reference_channels'] = list(refChannels[pmu0_plugs_dict[key]])
        currentMeasExists = False
        localSratio = loadScaling
    elif actType == 'modbus':
        cfg['rate'] = rate
        cfg['local_channels'] = list(pmu123PChannels[pmu123P_plugs_dict[key]])
        cfg['reference_channels'] = list(refChannels[pmu0_plugs_dict[key]]) #made these back into lists in case thats how gabes code expects it
        currentMeasExists = False
        localSratio = CILscaling
    else:
        error('actType Error')
    cfg['spbc'] = SPBCname
    timesteplength = cfg['rate']
    lpbcdict[key] = lpbcwrapper(cfg, key, testcase, nphases, act_idxs, actType, plug_to_phase_idx, timesteplength, currentMeasExists, localSratio) #Every LPBC will have its own step that it calls on its own

run_loop() #defined in XBOSProcess




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
