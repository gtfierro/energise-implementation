

from pyxbos.process import run_loop, config_from_file #https://github.com/gtfierro/xboswave/blob/master/python/pyxbos/pyxbos/process.py
from pyxbos.drivers import pbc  #https://github.com/gtfierro/xboswave/tree/master/python/pyxbos/pyxbos/drivers
# above imports LPBCProcess, SPBCProcess, EnergiseMessage, LPBCStatus, LPBCCommand, SPBC, EnergiseError
import sys
import matplotlib.pyplot as plt
import os #HERE for saving plots
#from pathlib import Path # https://medium.com/@ageitgey/python-3-quick-tip-the-easy-way-to-deal-with-file-paths-on-windows-mac-and-linux-11a072b58d5f
import numpy as np
import pandas as pd
import time as pytime
import warnings
import logging
import requests
from requests_futures.sessions import FuturesSession
from pymodbus.client.sync import ModbusTcpClient as ModbusClient

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
Did switch the sign of self.Icomp_pu, which is fed into the impedance estimator only
Did not switch the sign of Pact and Qact (which are positive out of network), or Pcmd and Qcmd (which are positive into the network)
Switched signs of Pact and Qact that are fed into check saturation and ICDI (which still communicates to SPBC using postive into network convention)
inverters are positive P out of the network for batt commands (positive P into the network for inverter-limiting commands)
inverters are positive Q into the netowk (because of how PF is calculated)
load_cmd is still postive into the network (for just P)
modbus is positive out of the network (switched internally)
'''

#HHERE put in t

#to use session.get for parallel API commands you have to download futures: pip install --user requests-futures

class lpbcwrapper(pbc.LPBCProcess): #this is related to super(), inherits attributes and behaviors from pbc.LPBCProcess (which is a wrapper for XBOSProcess)
    def __init__(self, cfg, busId, testcase, nphases, act_idxs, actType, plug_to_phase_idx, timesteplength, currentMeasExists, localSratio=1, localVratio=1, ORT_max_kVA=500, VmagScaling=1):
        super().__init__(cfg) #cfg goes to LPBCProcess https://github.com/gtfierro/xboswave/blob/master/python/pyxbos/pyxbos/drivers/pbc/pbc_framework.py

        print('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
        print(f'Building LPBC for performance node {busId}')
        # INITIALIZATION
        self.busId = busId
        self.timesteplength = timesteplength

        #HERE put in optional accumulator term for PI controller

        self.controllerType = 'LQR' #set controller to 'PI' or 'LQR'

        if self.controllerType == 'PI':
            # ang_scale = 1
            # mag_scale = 1
            # kp_ang = [0.01*ang_scale]
            # ki_ang = [0.3*ang_scale]
            # kp_mag = [0.01*mag_scale]
            # ki_mag = [0.3*mag_scale]
            self.controller = PIcontroller(nphases, kp_ang, ki_ang, kp_mag, ki_mag)

        elif self.controllerType == 'LQR':
            '''
            kV and kVA base are recieved in targetExtraction, which is called by step.
            when LPBC is created, step hasnt been called yet.
            so I created ZeffkinitInPU so that LQR can wait for the first target from the SPBC to set its Zeffkinit

            Controller feedback:
            controller gets a pu voltage, all the internal controller comps are done in pu, and the output is a pu power
            output power is multiplied by localSbase then Sratio, so effectively networkSbase (networkSbase determines how the power injection affects the local voltage, not localSbase)
            so the Zeff that is used should be the nonPU Zeff, divided by Zbase = Vbase^2/networkSbase

            Zestimation:
            current that is measured is based on localSbase, not networkSbase
            so the current measurement used to estimate Z should use localSbase
            (current meas has not accounted for inverter offset hack to reduce oscillations)
            '''
            self.usingNonpuZeff = 0 #setting this to 0 loads the saved pu Zeffk, to 1 loads the non pu Zeffk and waits for the first SPBC target to set the pu Zeffk
            self.ZeffkestinitHasNotBeenInitialized = 1 #only useful if self.usingNonpuZeff = 1, necessary bc KVA base is not received until first packet is received from the SPBC
            if self.usingNonpuZeff:
                ZeffkinitInPU = 0
                Zeffkpath = 'networkImpedanceModels/Zeffks/' + str(testcase) + '/notPU' + '/Zeffk_bus' + str(busId) + '.csv' #alternative
                # if testcase == 'manual': #HERE for debugging, assumes 13bal is used
                #     Zeffkpath = 'networkImpedanceModels/Zeffks/' + '13bal' + '/notPU' + '/Zeffk_bus' + str(busId) + '.csv' #alternative
            else:
                ZeffkinitInPU = 1
                Zeffkpath = 'networkImpedanceModels/Zeffks/' + str(testcase) + '/PU' + '/Zeffk_bus' + str(busId) + '.csv'
                # if testcase == 'manual': #HERE for debugging, assumes 13bal is used
                #     Zeffkpath = 'networkImpedanceModels/Zeffks/' + '13bal' + '/PU' + '/Zeffk_bus' + str(busId) + '.csv'
            Zeffk_df = pd.read_csv(Zeffkpath, index_col=0) #index_col=0 bc of how Im saving the df (should have done index = false)
            Zeffk_df = Zeffk_df.apply(lambda col: col.apply(lambda val: complex(val.strip('()')))) #bc data is complex
            Zeffk_init = np.asmatrix(Zeffk_df.values)

            #for logging Zeff estimation error
            # self.ZeffkError = []
            # self.GtMag = []
            if self.usingNonpuZeff == 0:
                self.ZeffkTru = Zeffk_init #self.ZeffkTru is an attribute of lpbcwrapper rather than the LQR controller bc the LQR doesnt know ZeffkTru (wrapper wouldnt either, in actual implementations)
            #else wait till Zbase is  #HERE will assigning a self. later create an error?

            #for testing the Zeffestimator
            # self.Zeffk_init_mult = .5
            # self.Zeffk_init_mult = 2
            self.Zeffk_init_mult = 1
            Zeffk_init = Zeffk_init*self.Zeffk_init_mult
            # print(f'Zeffk_init (PU) bus {busId}: ', Zeffk_init)
            ######################## LQR Controller Parameters #######################
            #General controller parameters
            linearizeplant = 1 #determines how the (V-V0) voltage is converted into an eq power injection

            #LQR parameters
            #Q matrix determines the penalties placed on state errors
            #first 3 states are the mag error at a given time step for each phase,
            #4 thru 6 are the ang error, 7 thru 9 are integrated mag error, 10 thru 12 are integrated ang error
            Qcost = np.asmatrix(np.eye(12))
            # Qcost[3:6,3:6] = Qcost[3:6,3:6]*10 #penalize ang state error
            # Qcost[9:12,9:12] = Qcost[9:12,9:12]*10  #penalize integrated ang error
            Qcost[6:12,6:12] = Qcost[6:12,6:12]*10 #penalize all integrated state error

            #R matrix determines the penalties placed on control effort
            #the first 3 are P control effort for each state, the second 3 are Q control effort
            Rcost = np.asmatrix(np.eye(6)) #control cost (smaller control cost gets to setpoint faster)
            # Rcost[3:,3:] = Rcost[3:,3:]**1e-1 #for cheap Q
            Rcost = Rcost*1e-1 #for cheap P and Q

            #DOBC parameters
            #The disturance observer cancels the affect of the other loads on the system (internal loop to the LQR's outer loop)
            cancelDists = 1 #setting this to 0 turns the disturbance cancelation off
            lpAlpha = .1 # low pass filter alpha for the disturbance estimator, larger alpha changes the disturbance estimate faster but is more noise sensitive
            # lpAlpha = .5

            #REIE parameters
            est_Zeffk = 0 #if this is set to 1 the effective impedance will be estimated online and used to update the LQR controller (by changing the network (plant) model)
            # lam = .99 # 0 < lam < 1, smaller lam changes state faster (more noise sensitive)
            lam = .95
            # lam = .5
            GtInitScale = 1
            # GtInitScale = 10
            controllerUpdateCadence = 1 #this is the cadence (of timesteps) with which K is updated

            if est_Zeffk:
                Gt = np.asmatrix(np.eye(3))*(1+1j)*GtInitScale
            else:
                Gt = None

            assert nphases == 3, 'LQR controller has only been set up for 3 phases at the moment'
            self.useRelativeMeas = 0 #default is 0. setting to 1 runs LQR with relative V measurements rather than nonRelative V measurements (still uses relative Vcomp)
            self.controller = LQRcontroller(busId,nphases,timesteplength,Qcost,Rcost,Zeffk_init,est_Zeffk,cancelDists,currentMeasExists,lpAlpha,lam,Gt,controllerUpdateCadence,linearizeplant,ZeffkinitInPU)
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

        #all angles should be in radians
        self.Vang_notRelative = np.asarray([np.NaN]*nphases) #The notRelative angles are created by subtracting the angle timestamp for the FIRST phase from ALL the phases, which will give angles seperated by ~120degrees.
        self.Vang_relative = np.asarray([np.NaN]*nphases) #relative angles have the reference angle (timestamp) for each phase subtracted for each phase, which will NOT give angles seperated by ~120degrees.
        self.Vmag = np.zeros(nphases)
        self.Vmag_pu = np.zeros(nphases)
        self.Vmag_relative = np.zeros(nphases)
        self.Vmag_relative_pu = np.zeros(nphases)
        self.phasor_error_ang = np.zeros(nphases)
        self.phasor_error_mag_pu = np.zeros(nphases)
        self.VmagRef = np.zeros(nphases) #rename these V0mag and V0ang at some point
        self.VmagRef_pu = np.zeros(nphases)
        self.VangRef = np.zeros(nphases)

        #Just need to decide what to call unintialized values (probably np.zero if more than 1 dimension)
        #Targets received from SPBC, right now VmagTarg as relative not abosolute
        self.VangTarg_relative = 'initialize' #intialized the first time a phasor_target packet comes from the SPBC, control loop isnt run until a packet is received
        #VangTarg_relative subtracts the reference nodes angle for each phase from each phase, so the realtive angles are all around 0 (rather than [0, -120, 120])
        self.VmagTarg_pu = 'initialize' #all angles should be in radians
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
        self.Iang_relative = 'initialize'
        self.Iang_notRelative = 'initialize'
        self.Imag = np.zeros(nphases)
        self.Icomp_est = np.zeros(nphases,dtype=np.complex_)
        self.Icomp_pu_est = np.zeros(nphases,dtype=np.complex_)
        self.Icomp = np.zeros(nphases,dtype=np.complex_)
        self.Icomp_pu = np.zeros(nphases,dtype=np.complex_)

        #saturation variables
        self.sat_arrayP = np.ones(nphases) #logic vectors which are 1 if a given phase is not saturated, and zero if it is
        self.sat_arrayQ = np.ones(nphases) #if no current measurements, then these will just stay zero and saturated == 0
        self.Pmax_pu = np.asarray([np.NaN] * nphases) #this signal is used by the SPBC if ICDI is true, otherwise its a nan
        self.Qmax_pu = np.asarray([np.NaN] * nphases)
        self.saturationCounterLimit = 5
        self.Psat = np.ones((nphases, self.saturationCounterLimit)) #set of sat_arrayPs
        self.Qsat = np.ones((nphases, self.saturationCounterLimit))
        self.ICDI_sigP = np.zeros((nphases, 1), dtype=bool) #I Cant Do It signal, defaulted to zero (that it can do it)
        self.ICDI_sigQ = np.zeros((nphases, 1), dtype=bool)

        #phasor calc
        self.local_time_index = [np.NaN]*nphases
        self.ref_time_index = [np.NaN]*nphases

        self.nPhasorReadings = 40 #HHHERE for debugging
        # self.nPhasorReadings = 120 # 150 # 100  # number of time measurements that phasorV_calc looks into the past to find a match
        self.pmuTimeWindow = 2000000 #in ns, 2000000 is 2 ms #allowable time window for phasor measurements to be considered concurrent

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
        self.mode = 1 #How we control inverters mode 1: PV as disturbance, mode 2: PV calculated, mode 3: PV only
        self.batt_cmd = np.zeros(nphases) #battery commands are given in watts
        self.invPperc_ctrl = np.zeros(nphases) #inverter P commnads are given as a percentage of inv_s_max
        self.load_cmd = np.zeros(nphases) #load commands are given in watts
        self.P_PV = np.zeros(nphases)
        self.pf_ctrl = np.ones(nphases)
        # self.flexgrid = Flexgrid_API(inv_ids=[1, 2, 3], portNames=['COM3'], baudrate=115200, parallel=False, safety=True,
        #                         debug=False, ComClient=ModbusRTUClient)
        self.inv_Pmax = 7000 #check with Maxime
        self.inv_Qmax = 5000 #check with Maxime
        self.offset_mode = 2 # set to 1 for remainder offset, 2 for percentage offset, 0 for no offset

        IP = '131.243.41.14'
        PORT = 504
        self.client = ModbusClient(IP, port=PORT)

        self.VmagScaling = VmagScaling #this is a hack to get flexlab to work. default to 1

        #vars for plots
        self.controlStepsTaken_counter = 0
        self.testcase = cfg['testcase']
        self.saveVmagandangPlot = 1
        self.saveZesterrorPlot = 1
        # self.HistLength = 100
        self.HistLength = 10
        self.VmagHist = np.zeros((self.nphases,self.HistLength))
        self.VangHist = np.zeros((self.nphases,self.HistLength))
        self.ZeffkErrorHist = np.zeros(self.HistLength)
        self.GtMagHist = np.zeros(self.HistLength)

        self.P_implemented_PU = None #to account for commands hitting the upper limits of an actuator
        self.Q_implemented_PU = None

        self.perturbPowerCommand = 0
        self.perturbScale = .1


    def targetExtraction(self,phasor_target):
        #5/28/20 SPBC (only) sends relative magnitude and angle targets (relative to the nominal reference, though SPBC does get the actual ref voltage, so that could be used later)
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
                print('no phase found for target ' + str(i) + 'using phase A')
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

    '''
    Think this is how the PMUs send data:
    Each PMU measurement comes with a time stamp and an angle measurement.
    The timestamps have to be aligned before the angle difference is taken
    The angle measurement that comes from the PMU has an arbitrary refernce. To get meaningful angle measurements, you have to decide on a reference.
    (Still deciding whether the arbitrary is the first phase for a given controller. Could make it always phase A, but reference_phasors would always have to contain phase A.)
    V_ang_ref_firstPhase is special because it is chosen as angle = 0, and subtracted (sometimes implicitly) from all other angles.
    self.Vang_notRelative is named poorly (by me), just means that V_ang_ref_firstPhase is subtracted, rather than V_ang_ref[phase], so the angles will be relative to [0,0,0] rather than [0,-120,120]
    '''
    def phasorV_calc(self, local_phasors, reference_phasors, nphases, plug_to_V_idx):
        # Initialize
        #ordered_local and ref are the PMU meas data sets from the local and ref PMUs, respectively. First dim is phase, second is dataWindowLength.
        # ordered_local = [None] * nphases # makes a list nphases-long, similar to np.zeros(nphases), but a list
        # ref = [None] * nphases
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
            #HERE small chance theres a problem here w copying a mutable data type and not using .copy()

        #this was creating issues when intitial phasor reading wasnt correct
        # if self.Vang_relative == 'initialize':
        #     self.Vang_relative = np.zeros(nphases)
        #     # loops through every phase with actuation
        #     for phase in range(nphases): #phases descrived by a,b,c ordering, but not necessarily a,b,c, all angles are base zero (ie not base -2pi/3 for phase B) bec they are relative angles
        #         # Initialize: Extract measurements from most recent timestamps only for first iteration
        #         V_mag_local = ordered_local[phase][-1]['magnitude']
        #         V_ang_local = ordered_local[phase][-1]['angle'] - self.ametek_phase_shift
        #         V_mag_ref = ref[phase][-1]['magnitude']
        #         V_ang_ref = ref[phase][-1]['angle']
        #         self.Vang_relative[phase] = np.radians(V_ang_local - V_ang_ref)
        #         self.Vmag[phase] = V_mag_local
        #         self.VmagRef[phase] = V_mag_ref
        #         self.Vmag_relative[phase] = V_mag_local - V_mag_ref

        Vmag = np.asarray([np.NaN]*nphases)
        VmagRef = np.asarray([np.NaN]*nphases)
        Vmag_relative = np.asarray([np.NaN]*nphases)

        VmagSum = np.zeros(nphases)
        VmagCount = np.zeros(nphases)
        VmagRefSum = np.zeros(nphases)
        VmagRefCount = np.zeros(nphases)
        for phase in range(nphases):
            # loops through every ordered_local uPMU reading
            for local_packet in ordered_local[phase]:
                Vmagi = local_packet['magnitude']
                Vmagi = Vmagi * self.VmagScaling
                if Vmagi is None:
                    print('Vmagi is None')
                elif np.isnan(Vmagi):
                    print('Vmagi is NaN')
                elif Vmagi == 0:
                    print('Vmagi is 0')
                else:
                    VmagSum[phase] += Vmagi
                    VmagCount[phase] += 1
            for ref_packet in ref[phase]:
                VmagRefi = ref_packet['magnitude']
                VmagRefi = VmagRefi * self.VmagScaling
                if VmagRefi is None:
                    print('VmagRefi is None')
                elif np.isnan(VmagRefi):
                    print('VmagRefi is NaN')
                elif VmagRefi == 0:
                    print('VmagRefi is 0')
                else:
                    VmagRefSum[phase] += VmagRefi
                    VmagRefCount[phase] += 1
            Vmag[phase] = VmagSum[phase]/VmagCount[phase]
            VmagRef[phase] = VmagRefSum[phase]/VmagRefCount[phase]
            Vmag_relative[phase] = Vmag[phase] - VmagRef[phase]

        print('::::::::::::::::::::::::::::::::::::::::::::::::::::::::')
        print('len(local_phasors[plug]) ', len(local_phasors[plug]))
        print('len(reference_phasors[plug]) ', len(reference_phasors[plug]))

        print('ordered_local[0][0][time] - ordered_local[0][-1][time] ', int(ordered_local[0][0]['time']) - int(ordered_local[0][-1]['time']))
        print('ref[0][0][time] - ref[0][-1][time] ', int(ref[0][0]['time']) - int(ref[0][-1]['time']))

        print('VmagCount ', VmagCount)
        print('VmagRefCount ', VmagRefCount)
        print('Vmag ', Vmag)
        print('VmagRef ', VmagRef)
        print('Vmag_relative ', Vmag_relative)
        print('::::::::::::::::::::::::::::::::::::::::::::::::::::::::')

        local_time_index = [np.NaN]*nphases
        ref_time_index = [np.NaN]*nphases

        Vang_notRelative = np.asarray([np.NaN]*nphases)
        Vang_relative = np.asarray([np.NaN]*nphases)
        VangRef = np.asarray([np.NaN]*nphases)
        # V_ang_ref_firstPhase = [np.NaN]
        V_ang_ref_firstPhase = np.asarray([np.NaN]*nphases) #using nphase-long version for back-compatibility, they should all be the same

        VangCount = np.zeros(nphases)
        Vang_notRelativeSum = np.zeros(nphases)
        # Vang_notRelativeCount = np.zeros(nphases)
        Vang_relativeSum = np.zeros(nphases)
        # Vang_relativeCount = np.zeros(nphases)
        VangRefSum = np.zeros(nphases)
        # VangRefCount = np.zeros(nphases)
        V_ang_ref_firstPhaseSum = np.zeros(nphases)
        # V_ang_ref_firstPhaseCount = np.zeros(nphases)

        #5/28/20 sets the first phase as the local base angle timestamp even if this phase is B or C
        #this is okay bc the local controller can just use 0 for its first angle (locally), even if that angle is phase is B or C
        #important thing is that the other notRelative angles are seperated by ~120degrees
        for phase in range(nphases):
            print('phase ', phase)
            refAngleUsedVec = np.zeros(dataWindowLength) # debug to check if any refs are used twice (they shouldnt be)
            # loops through every ordered_local uPMU reading starting from most recent
            for local_packet in reversed(ordered_local[phase]): #doesnt need ot be reversed when using averaging (as done now), but doesnt hurt
                # extract most recent ordered_local uPMU reading
                local_time = int(local_packet['time'])
                # loops though every reference uPMU reading starting from most recent
                i = 0
                for ref_packet in reversed(ref[phase]):
                    ref_time = int(ref_packet['time'])

                    # check timestamps of ordered_local and reference uPMU if within 2 ms
                    if abs(ref_time - local_time) <= self.pmuTimeWindow:
                        local_time_index[phase] = ordered_local[phase].index(local_packet) #saves and returns these so the current measurement can use the measurements from the same timestamps
                        ref_time_index[phase] = ref[phase].index(ref_packet)
                        # Extract measurements from closest timestamps
                        # V_ang_local = ordered_local[phase][local_time_index[phase]]['angle'] - self.ametek_phase_shift
                        # V_ang_ref = ref[phase][ref_time_index[phase]]['angle']
                        # V_ang_ref_firstPhaseTemp = ref[0][ref_time_index[phase]]['angle']
                        V_ang_local = self.PhasorV_ang_wraparound_1d(ordered_local[phase][local_time_index[phase]]['angle'] - self.ametek_phase_shift)
                        V_ang_ref = self.PhasorV_ang_wraparound_1d(ref[phase][ref_time_index[phase]]['angle'])
                        V_ang_ref_firstPhaseTemp = self.PhasorV_ang_wraparound_1d(ref[0][ref_time_index[phase]]['angle'])
                        print('V_ang_local ', V_ang_local)
                        print('V_ang_ref ', V_ang_ref)
                        print('V_ang_ref_firstPhaseTemp ', V_ang_ref_firstPhaseTemp)

                        # V_ang_ref_firstPhase = ref[0][ref_time_index[phase]]['angle'] #this can be thought of as the local base angle timestamp
                        # if V_ang_ref_firstPhase == np.NaN or V_ang_ref_firstPhase == None: #(could put in a better check here, eg is the angle in a reasonable range)
                        V_ang_ref_firstPhaseSum[phase] += V_ang_ref_firstPhaseTemp #because each phase (of the current meas) needs a V_ang_ref_firstPhase
                        if V_ang_ref_firstPhase[phase] == np.NaN or V_ang_ref_firstPhase[phase] == None: #(could put in a better check here, eg is the angle in a reasonable range)
                            print('WARNING: issue getting a nonRelative voltage angle. This will mess up the LQR controller.')

                        Vang_relativeSum[phase] += np.radians(V_ang_local - V_ang_ref)
                        Vang_notRelativeSum[phase] += np.radians(V_ang_local - V_ang_ref_firstPhaseTemp)
                        VangRefSum[phase] += np.radians(V_ang_ref - V_ang_ref_firstPhaseTemp)
                        VangCount[phase] += 1
                        if refAngleUsedVec[i] == 1 and phase == 0: #debug
                            print(f'WARNING, this ref angle {i} was already used')
                        refAngleUsedVec[i] = 1

                        flag[phase] = 0
                        #for debugging
                        if phase == 0:
                            print('i used ', i)
                            print(f'ref,local,diff: {ref_time},{local_time},{(ref_time-local_time)/1e6}')
                        # break # dont want this break when doing averaging

                    i += 1
                # if flag[phase] == 0:
                #     break
            if flag[phase] == 1:
                print('No timestamp found bus ' + str(self.busId) + ' phase ' + str(phase))
                Vmeas_all_phases = 0
            else:
                Vang_notRelative[phase] = Vang_notRelativeSum[phase]/VangCount[phase]
                Vang_relative[phase] = Vang_relativeSum[phase]/VangCount[phase]
                VangRef[phase] = VangRefSum[phase]/VangCount[phase]
                V_ang_ref_firstPhase[phase] = V_ang_ref_firstPhaseSum[phase]/VangCount[phase]

        print('Vang_notRelative ', Vang_notRelative)
        print('Vang_relative ', Vang_relative)
        print('VangRef ', VangRef)
        print('V_ang_ref_firstPhase ', V_ang_ref_firstPhase)
        print('VangCount ', VangCount)
        print('::::::::::::::::::::::::::::::::::::::::::::::::::::::::')
        return (Vang_notRelative,VangRef,Vang_relative,Vmag,VmagRef,Vmag_relative, V_ang_ref_firstPhase, dataWindowLength, Vmeas_all_phases) #returns the self. variables bc in case a match isnt found, they're already initialized


        # older version that didnt take average of angles
        # # loops through each set of voltage measurements for each phase
        # local_time_index = [np.NaN]*nphases
        # ref_time_index = [np.NaN]*nphases
        # #below isnt needed if you switch back to using self. values
        # Vang_notRelative = np.asarray([np.NaN]*nphases)
        # VangRef = np.asarray([np.NaN]*nphases)
        # Vang_relative = np.asarray([np.NaN]*nphases)
        # # Vmag = np.asarray([np.NaN]*nphases)
        # # VmagRef = np.asarray([np.NaN]*nphases)
        # # Vmag_relative = np.asarray([np.NaN]*nphases)
        #
        # # V_ang_ref_firstPhase = [np.NaN]
        # V_ang_ref_firstPhase = [np.NaN]*nphases #using this for back-compatibility
        #
        # #5/28/20 sets the first phase as the local base angle timestamp even if this phase is B or C
        # #this is okay bc the local controller can just use 0 for its first angle (locally), even if that angle is phase is B or C
        # #important thing is that the other notRelative angles are seperated by ~120degrees
        # for phase in range(nphases):
        #     # loops through every ordered_local uPMU reading starting from most recent
        #     for local_packet in reversed(ordered_local[phase]):
        #         # extract most recent ordered_local uPMU reading
        #         local_time = int(local_packet['time'])
        #         # loops though every reference uPMU reading starting from most recent
        #         for ref_packet in reversed(ref[phase]):
        #             ref_time = int(ref_packet['time'])
        #
        #             #print(f'ref,local,diff: {ref_time},{local_time},{(ref_time-local_time)/1e6}')
        #
        #             # check timestamps of ordered_local and reference uPMU if within 2 ms
        #             if abs(ref_time - local_time) <= self.pmuTimeWindow:
        #                 local_time_index[phase] = ordered_local[phase].index(local_packet) #saves and returns these so the current measurement can use the measurements from the same timestamps
        #                 ref_time_index[phase] = ref[phase].index(ref_packet)
        #                 # Extract measurements from closest timestamps
        #                 # V_mag_local = ordered_local[phase][local_time_index[phase]]['magnitude']
        #                 # V_mag_ref = ref[phase][ref_time_index[phase]]['magnitude']
        #                 # Vmag[phase] = V_mag_local
        #                 # VmagRef[phase] = V_mag_ref
        #                 # Vmag_relative[phase] = V_mag_local - V_mag_ref
        #
        #                 V_ang_local = ordered_local[phase][local_time_index[phase]]['angle'] - self.ametek_phase_shift
        #                 V_ang_ref = ref[phase][ref_time_index[phase]]['angle']
        #                 # V_ang_ref_firstPhase = ref[0][ref_time_index[phase]]['angle'] #this can be thought of as the local base angle timestamp
        #                 V_ang_ref_firstPhase[phase] = ref[0][ref_time_index[phase]]['angle'] #because each phase (of the current meas) needs a V_ang_ref_firstPhase
        #                 if V_ang_ref_firstPhase == np.NaN or V_ang_ref_firstPhase == None: #(could put in a better check here, eg is the angle in a reasonable range)
        #                     print('WARNING: issue getting a nonRelative voltage angle. This will mess up the LQR controller.')
        #
        #                 Vang_relative[phase] = np.radians(V_ang_local - V_ang_ref)
        #                 Vang_notRelative[phase] = np.radians(V_ang_local - V_ang_ref_firstPhase[phase])
        #                 VangRef[phase] = np.radians(V_ang_ref - V_ang_ref_firstPhase[phase])
        #                 # Vang_notRelative[phase] = np.radians(V_ang_local - V_ang_ref_firstPhase)
        #                 # VangRef[phase] = np.radians(V_ang_ref - V_ang_ref_firstPhase)
        #                 flag[phase] = 0
        #                 break
        #         if flag[phase] == 0:
        #             break
        #     if flag[phase] == 1:
        #         print('No timestamp found bus ' + str(self.busId) + ' phase ' + str(phase))
        #         Vmeas_all_phases = 0
        # return (Vang_notRelative,VangRef,Vang_relative,Vmag,VmagRef,Vmag_relative, local_time_index, ref_time_index, V_ang_ref_firstPhase, dataWindowLength, Vmeas_all_phases) #returns the self. variables bc in case a match isnt found, they're already initialized


        # #alternative to find a timestamp at which all voltages are aligned, rather than finding presumably different time steps for each phase
        # #decided not to implement this bc its less flexible than the option above, which appears to be working well at the moment
        # # loops through every ordered_local uPMU reading starting from most recent
        # #each of these would work, I think
        # # for local_packet in reversed(ordered_local[phase]): then i = ordered_local[phase].index(local_packet)
        # # for i, local_packet in reversed(list(enumerate(ordered_local[phase]))):
        # for i in reversed(range(len(ordered_local[0]))):
        #     # extract most recent ordered_local uPMU reading
        #     for phase in range(nphases):
        #         local_time[phase] = int(ordered_local[phase][i]['time'])
        #     # loops though every reference uPMU reading starting from most recent
        #     # for ref_packet in reversed(ref[phase]):
        #     if nphases > 1 and (local_time[0] != local_time[1]):
        #         print('WARNING local phase times not matching up')
        #     if nphases > 2 and (local_time[1] != local_time[2]):
        #         print('WARNING local phase times not matching up')
        #     for i in reversed(range(len(ref[0]))):
        #         for phase in range(nphases):
        #             ref_time[phase] = int(ref[phase][i]['time'])
        #         if nphases > 1 and (ref_time[0] != ref_time[1]):
        #             print('WARNING ref phase times not matching up')
        #         if nphases > 2 and (ref_time[1] != ref_time[2]):
        #             print('WARNING ref phase times not matching up')
        #
        #         #print(f'ref,local,diff: {ref_time},{local_time},{(ref_time-local_time)/1e6}')
        #
        #         # check timestamps of ordered_local and reference uPMU if within 2 ms
        #         # if abs(ref_time - local_time) <= self.pmuTimeWindow:
        #         #     local_time_index[phase] = ordered_local[phase].index(local_packet) #saves and returns these so the current measurement can use the measurements from the same timestamps
        #         #     ref_time_index[phase] = ref[phase].index(ref_packet)
        #         if all(abs(ref_time - local_time) <= self.pmuTimeWindow):
        #             #dont need seperate time indeces for this verion, which checks that all time indeces are lined up for the given time, but leaving in for back-compatibility
        #             for phase in range(nphases):
        #                 local_time_index[phase] = local_time[phase] #saves and returns these so the current measurement can use the measurements from the same timestamps
        #                 ref_time_index[phase] = ref_time[phase]
        #                 # Extract measurements from closest timestamps
        #                 V_mag_local = ordered_local[phase][local_time_index[phase]]['magnitude']
        #                 V_ang_local = ordered_local[phase][local_time_index[phase]]['angle'] - self.ametek_phase_shift
        #                 V_mag_ref = ref[phase][ref_time_index[phase]]['magnitude']
        #                 V_ang_ref = ref[phase][ref_time_index[phase]]['angle']
        #                 # V_ang_ref_firstPhase = ref[0][ref_time_index[phase]]['angle'] #this can be thought of as the local base angle timestamp
        #                 V_ang_ref_firstPhase[phase] = ref[0][ref_time_index[phase]]['angle'] #for back-compatibility (all phases)
        #                 if V_ang_ref_firstPhase == np.NaN or V_ang_ref_firstPhase == None: #(could put in a better check here, eg is the angle in a reasonable range)
        #                     print('WARNING: issue getting a nonRelative voltage angle. This will mess up the LQR controller.')
        #
        #                 # calculates relative phasors
        #                 # self.Vang_relative[phase] = np.radians(V_ang_local - V_ang_ref)
        #                 # self.Vmag[phase] = V_mag_local
        #                 # self.VmagRef[phase] = V_mag_ref
        #                 # self.Vmag_relative[phase] = V_mag_local - V_mag_ref
        #                 # self.Vang_notRelative[phase] = np.radians(V_ang_local - V_ang_ref_firstPhase[phase])
        #                 # self.VangRef[phase] = np.radians(V_ang_ref - V_ang_ref_firstPhase[phase]) #this is the angle that, when added to self.Vang_relative, gives self.Vang_notRelative. Will always be zero for the first phase, and close to [0, -120, 120] for a 3 phase node.
        #                 # # self.Vang_notRelative[phase] = np.radians(V_ang_local - V_ang_ref_firstPhase)
        #                 # # self.VangRef[phase] = np.radians(V_ang_ref - V_ang_ref_firstPhase)
        #                 #uncomment above and change the return statement if you want the default to be to use the previous V measurment when V measurements are not successfully calculated for each phase
        #                 Vang_relative[phase] = np.radians(V_ang_local - V_ang_ref)
        #                 Vmag[phase] = V_mag_local
        #                 VmagRef[phase] = V_mag_ref
        #                 Vmag_relative[phase] = V_mag_local - V_mag_ref
        #                 Vang_notRelative[phase] = np.radians(V_ang_local - V_ang_ref_firstPhase[phase])
        #                 VangRef[phase] = np.radians(V_ang_ref - V_ang_ref_firstPhase[phase])
        #                 # Vang_notRelative[phase] = np.radians(V_ang_local - V_ang_ref_firstPhase)
        #                 # VangRef[phase] = np.radians(V_ang_ref - V_ang_ref_firstPhase)
        #             flag = 0
        #             break
        #     if flag == 0:
        #         break
        # if flag == 1:
        #     print('No timestamp found bus ' + str(self.busId) + ' phase ' + str(phase))
        #     Vmeas_all_phases = 0
        #     #self. vars are assigned and returned so that if a match isnt found, it returns the previous match
        # # return (self.Vang_notRelative,self.VangRef,self.Vang_relative,self.Vmag,self.VmagRef,self.Vmag_relative, local_time_index, ref_time_index, V_ang_ref_firstPhase, dataWindowLength, Vmeas_all_phases) #returns the self. variables bc in case a match isnt found, they're already initialized
        # return (Vang_notRelative,VangRef,Vang_relative,Vmag,VmagRef,Vmag_relative, local_time_index, ref_time_index, V_ang_ref_firstPhase, dataWindowLength, Vmeas_all_phases) #returns the self. variables bc in case a match isnt found, they're already initialized


    def phasorI_calc(self, dataWindowLength, local_phasors, reference_phasors, nphases, plug_to_V_idx):
        #uses the same time indeces and votlage reference from the voltage search
        # Initialize
        ordered_local = [0] * nphases # makes a list nphases-long, similar to np.zeros(nphases), but a list
        ref = [0] * nphases #dont think this is ever needed, current is not a relative measurement the way voltage is

        for plug in range(nphases): #this will just read the voltage measurements cause its nphases long, even if local_phasors also has current measurements
            # if len(local_phasors[plug]) > self.nPhasorReadings:
            #     dataWindowLength = self.nPhasorReadings
            # else:
            #     dataWindowLength = len(local_phasors[plug])
            phase_idx = plug_to_V_idx[plug]
            # the + nphases gives the current rather than the voltage measurements
            ordered_local[phase_idx] = local_phasors[plug + nphases][-dataWindowLength:] #this orders local in A,B,C phase order (ref is assumed ot be in A,B,C order)
            # no + nphases for ref bc you WANT the voltage ref
            ref[plug] = reference_phasors[plug][-dataWindowLength:] #from dataWindowLength back to present, puts Lx2 entries in each entry of local, x2 is for magnitude and phase

        print(';;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;')
        print('Current ordered_local[phase][0][time] ', ordered_local[phase][0]['time'])
        print('Current ordered_local[phase][-1][time] ', ordered_local[phase][-1]['time'])
        print('Current ref[phase][0][time] ', ref[phase][0]['time'])
        print('Current ref[phase][-1][time] ', ref[phase][-1]['time'])

        ImagSum = np.zeros(nphases)
        ImagCount = np.zeros(nphases)
        for phase in range(nphases):
            # loops through every ordered_local uPMU reading
            for local_packet in ordered_local[phase]:
                Imagi = local_packet['magnitude']
                if Imagi is None:
                    print('Imagi is None')
                elif np.isnan(Imagi):
                    print('Imagi is NaN')
                elif Imagi == 0:
                    print('Imagi is 0')
                else:
                    ImagSum[phase] += Imagi
                    ImagCount[phase] += 1
            Imag[phase] = ImagSum[phase]/ImagCount[phase]
        print('ImagCount ', ImagCount)
        print('Imag ', Imag)
        print(';;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;')

        Iang_notRelative = [np.NaN] * nphases

        IangCount = np.zeros(nphases)
        Iang_notRelativeSum = np.zeros(nphases)

        refAngleUsedVec = np.zeros(dataWindowLength) # to check if any refs are used twice (they shouldnt be)
        #for ref_i, ref_packet in enumerate(reversed(ref[phase])):

        for phase in range(nphases):
            # loops through every ordered_local uPMU reading starting from most recent
            for local_packet in reversed(ordered_local[phase]): #doesnt need ot be reversed when using averaging (as done now), but doesnt hurt
                # extract most recent ordered_local uPMU reading
                local_time = int(local_packet['time'])
                # loops though every reference uPMU reading starting from most recent
                for ref_packet in reversed(ref[phase]):
                    ref_time = int(ref_packet['time'])

                    # check timestamps of ordered_local and reference uPMU if within 2 ms
                    if abs(ref_time - local_time) <= self.pmuTimeWindow:
                        local_time_index[phase] = ordered_local[phase].index(local_packet) #saves and returns these so the current measurement can use the measurements from the same timestamps
                        ref_time_index[phase] = ref[phase].index(ref_packet)
                        # Extract measurements from closest timestamps
                        I_ang_local = self.PhasorV_ang_wraparound_1d(ordered_local[phase][local_time_index[phase]]['angle'] - self.ametek_phase_shift)
                        V_ang_ref_firstPhaseTemp = self.PhasorV_ang_wraparound_1d(ref[0][ref_time_index[phase]]['angle'])

                        if V_ang_ref_firstPhase[phase] == np.NaN or V_ang_ref_firstPhase[phase] == None: #(could put in a better check here, eg is the angle in a reasonable range)
                            print('WARNING: [in phasorI_calc] issue getting a nonRelative voltage angle. This will mess up the LQR controller.')

                        Iang_notRelativeSum[phase] += np.radians(I_ang_local - V_ang_ref_firstPhaseTemp)
                        IangCount[phase] += 1
                        # if refAngleUsedVec[i] == 1:
                        #     print('WARNING, this ref angle was already used')
                        # refAngleUsedVec[i] = 1

                        flag[phase] = 0
                        #for debugging
                        print(f'Current ref,local,diff: {ref_time},{local_time},{(ref_time-local_time)/1e6}')

            if flag[phase] == 1:
                print('PhasorI_calc: No timestamp found bus ' + str(self.busId) + ' phase ' + str(phase))
            else:
                Iang_notRelative[phase] = Iang_notRelativeSum[phase]/IangCount[phase]

        print('Iang_notRelative ', Iang_notRelative)
        print('IangCount ', IangCount)
        print(';;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;')

        # return self.Iang_notRelative, self.Iang_relative, self.Imag
        return Iang_notRelative, Imag

    #old version that didnt take average
    # # def phasorI_calc(self, local_time_index, ref_time_index, V_ang_ref_firstPhase, dataWindowLength, local_phasors, reference_phasors, nphases, plug_to_V_idx):
    # def phasorI_calc(self, local_time_index, ref_time_index, V_ang_ref_firstPhase, dataWindowLength, local_phasors, nphases, plug_to_V_idx):
    #     #uses the same time indeces and votlage reference from the voltage search
    #     # Initialize
    #     ordered_local = [0] * nphases # makes a list nphases-long, similar to np.zeros(nphases), but a list
    #     # ref = [0] * nphases #dont think this is ever needed, current is not a relative measurement the way voltage is
    #     Imag = [np.NaN] * nphases
    #     # Iang_relative = [np.NaN] * nphases
    #     Iang_notRelative = [np.NaN] * nphases
    #     for plug in range(nphases):
    #         phase_idx = plug_to_V_idx[plug] #assumes the current plugs are hooked up the same way
    #         ordered_local[phase_idx] = local_phasors[plug + nphases][-dataWindowLength:] #from dataWindowLength back to present, puts Lx2 entries in each entry of local, x2 is for magnitude and phase
    #         # ref[plug] = reference_phasors[plug + nphases][-dataWindowLength:] #plug + nphases selects the current data rather than the voltage data
    #
    #     for phase in range(nphases):
    #         # Extract measurements from closest timestamps
    #         I_ang_local = ordered_local[phase][local_time_index[phase]]['angle']
    #         # I_ang_ref = ref[phase][ref_time_index[phase]]['angle']
    #         # I_ang_ref_firstPhase = ref[0][ref_time_index[phase]]['angle'] # this is wrong, need to Vref[0]
    #         I_ang_ref_firstPhase = V_ang_ref_firstPhase[phase] # V_ang_ref_firstPhase[phase] = ref[0][ref_time_index[phase]]['angle'] #this is indexed by phase in case the different phase measurements use different time steps
    #
    #         # self.Iang_relative[phase] = np.radians(I_ang_local - I_ang_ref)  #uses self. so it defaults to previous value
    #         # self.Imag[phase] = ordered_local[phase][local_time_index[phase]]['magnitude']
    #         Iang_notRelative[phase] = np.radians(I_ang_local - I_ang_ref_firstPhase)
    #         Imag[phase] = ordered_local[phase][local_time_index[phase]]['magnitude']
    #
    #     # return self.Iang_notRelative, self.Iang_relative, self.Imag
    #     return Iang_notRelative, Imag


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
        ''' COMMENTED OUT FOR CIL TESTING ONLY!
        for plug in range(nphases):
            phase_idx = plug_to_V_idx[plug] #assumes plug to V map is the same for uPMUp123 voltage, uPMU123 current and uPMU123 voltage
            V_mag[phase_idx] = local_phasors[plug][-1]['magnitude'] #pulls out vmeas from uPMU123 not uPMUP123
            V_ang[phase_idx] = local_phasors[plug][-1]['angle']
            I_mag[phase_idx] = local_phasors[(nphases + plug)][-1]['magnitude'] # Check plugs!
            I_ang[phase_idx] = local_phasors[(nphases + plug)][-1]['angle'] # Check plugs!
            theta[phase_idx] = np.radians(V_ang[phase_idx] - I_ang[phase_idx]) #angle comes in in degrees, theta is calced for each phase, so there shouldnt be any 2pi/3 offsets
            # P = (VI)cos(theta), Q = (VI)sin(theta)
            Pact_kVA[phase_idx] = V_mag[phase_idx] * I_mag[phase_idx] * (np.cos(theta[phase_idx]))/1000
            Qact_kVA[phase_idx] = V_mag[phase_idx] * I_mag[phase_idx] * (np.sin(theta[phase_idx]))/1000
        '''
        return (Pact_kVA,Qact_kVA)


    def checkSaturationWoImeas(self, nphases, Vcomp, Pcmd_kVA, Qcmd_kVA,):
        # compare self.VcompPrev w Vcomp and if it keeps missing in the same direction declare that its saturated
        #could be confused by Q offset
        sat_arrayP = np.ones(nphases) # 1 indicates not saturated
        sat_arrayQ = np.ones(nphases)
        return sat_arrayP, sat_arrayQ


    def checkSaturation(self, nphases, Pact, Qact, Pcmd_kVA, Qcmd_kVA, P_PV):
        Pcmd = Pcmd_kVA * 1000
        Qcmd = Qcmd_kVA * 1000
        Pact_VA = Pact*1000
        Qact_VA = Qact*1000
        if self.actType == 'inverter':

            '''
            HAD TO COMMENT OUT AND MICKEY MOUSE SATURATION CHECK FOR CIL
            # find indicies where Pact + tolerance is less than Pcmd
            #indexP = np.where(abs(Pact_VA + (0.03 * Pcmd)) < abs(Pcmd))[0] #will be zero if Pcmd is zero
            print(f'PactVA: {Pact_VA}, P_PV: {P_PV}, Pact-P_PV+500: {abs(Pact_VA - P_PV)+500}, abs(Pcmd): {abs(Pcmd)}')
            indexP = np.where(abs(Pact_VA - P_PV) + 500 < abs(Pcmd))[0] #specific to step size of inverters
            # find indicies where Qact + tolerance is less than Qcmd
            #indexQ = np.where(abs(Qact_VA + (0.03 * Qcmd)) < abs(Qcmd))[0]
            print(f'QactVA+250: {abs(Qact_VA)+250}, abs(Qcmd): {abs(Qcmd)}')
            indexQ = np.where(abs(Qact_VA) + 250 < abs(Qcmd))[0]

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
                    '''
                    COMMENTED OUT FOR CIL TESTING
                    #self.Pmax_pu[phase] = Pact_pu[phase]
                    '''
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
                    '''
                    COMMENTED OUT FOR CIL TESTING
                    self.Qmax_pu[phase] = Qact_pu[phase]
                    '''
                    self.Qmax_pu[phase] = self.ORT_max_VA /(self.localkVAbase[phase] *1000)
                elif self.actType == 'load':
                    self.Qmax_pu[phase] = 0
                elif self.actType == 'modbus':
                    self.Qmax_pu[phase] = self.ORT_max_VA /(self.localkVAbase[phase] *1000)
            else:
                self.ICDI_sigQ[phase] = False
                self.Qmax_pu[phase] = np.NaN
        return (self.ICDI_sigP, self.ICDI_sigQ, self.Pmax_pu, self.Qmax_pu)


    def httptoInverters(self, nphases, act_idxs, Pcmd_kVA, Qcmd_kVA, Pact):
        # hostname: http://131.243.41.47:
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
        if self.mode == 2: #mode 2: PV calculated (from previous timestep)
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
        if self.mode == 3: #mode 3: PV only (no battery commands)
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
        responses = map(session.get, urls)
        results = [resp.result() for resp in responses]
        for i in range(nphases):
            if results[i].status_code == 200:
                commandReceipt[i] = 'success'
            else:
                commandReceipt[i] = 'failure'
        return commandReceipt


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
                urls.append(f"http://131.243.41.118:9090/control?group_id={group},P_ctrl={self.loadrack_manuallimit}")
            elif self.load_cmd[i] < 0:
                urls.append(f"http://131.243.41.118:9090/control?group_id={group},P_ctrl=0")
            else:
                urls.append(f"http://131.243.41.118:9090/control?group_id={group},P_ctrl={self.load_cmd[i]}")
        responses = map(session.get, urls)
        results = [resp.result() for resp in responses]
        for i in range(nphases):
            if results[i].status_code == 200:
                commandReceipt[i] = 'success'
            else:
                commandReceipt[i] = 'failure'
        return commandReceipt


    def modbustoOpal(self, nphases, Pcmd_kVA, Qcmd_kVA, ORT_max_VA, localSratio, client ):
        Pcmd_VA = -1 * (Pcmd_kVA * 1000) #sign negation is convention of modbus
        Qcmd_VA = -1 * (Qcmd_kVA * 1000) #sign negation is convention of modbus
        for phase in range(nphases):
            print(f'Opal Pcmd_VA[{phase}] : ' + str(Pcmd_VA[phase]))
            print(f'Opal Qcmd_VA[{phase}] : ' + str(Qcmd_VA[phase]))
            print('ORT_max_VA/localSratio : ' + str(ORT_max_VA/localSratio))
            if abs(Pcmd_VA[phase]) > ORT_max_VA/localSratio:
                print('WARNING Pcmd over Opal limit, using +/- max: ', np.sign(Pcmd_VA[phase]) * ORT_max_VA/localSratio)
                Pcmd_VA[phase] = np.sign(Pcmd_VA[phase]) * ORT_max_VA/localSratio
            if abs(Qcmd_VA[phase]) > ORT_max_VA/localSratio:
                print('WARNING Qcmd over Opal limit, using +/- max: ', np.sign(Qcmd_VA[phase]) * ORT_max_VA/localSratio)
                Qcmd_VA[phase] = np.sign(Qcmd_VA[phase]) * ORT_max_VA/localSratio
        id = 3
        # P,Q commands in W and VAR (not kilo)
        P_implemented_PU = Pcmd_VA/(self.localkVAbase*1000) #HERE bc Pcmd_VA = Pcmd_PU * self.localkVAbase * 1000
        Q_implemented_PU = Qcmd_VA/(self.localkVAbase*1000)

        if nphases == 3:
            P1, P2, P3 = abs(Pcmd_VA[0]), abs(Pcmd_VA[1]), abs(Pcmd_VA[2])
            Q1, Q2, Q3 = abs(Qcmd_VA[0]), abs(Qcmd_VA[1]), abs(Qcmd_VA[2])
        # TODO modbus only: manually change phase actuation on modbus here if needed on different phase
        elif nphases == 1:
            P1, P2, P3 = abs(Pcmd_VA[0]), 0, 0
            Q1, Q2, Q3 = abs(Qcmd_VA[0]), 0, 0

        elif nphases == 2: # Phase A, B only (change if needed)
            P1, P2, P3 = abs(Pcmd_VA[0]), abs(Pcmd_VA[1]), 0
            Q1, Q2, Q3 = abs(Qcmd_VA[0]), abs(Qcmd_VA[1]), 0

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

        elif nphases == 2: # Phase A, B only (change if needed)
            sign_base = 2 ** 5 * sign_vec[0] + 2 ** 4 * sign_vec[1] + 2 ** 3 * sign_vec[2] + 2 ** 2 * sign_vec[3]


        mtx = [P1, Q1, P2, Q2, P3, Q3, sign_base]
        print('mtx : ' + str(mtx))
        mtx_register = np.arange(1, 8).tolist()
        try:
            client.connect()
            # write switch positions for config
            for i in range(len(mtx)):
                client.write_registers(int(mtx_register[i]), int(mtx[i]), unit=id)
            result = 'sent'
        except Exception as e:
            result = ('exceptions', e)
        finally:
            client.close()
        return result, P_implemented_PU, Q_implemented_PU


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

    def PhasorV_ang_wraparound_1d(self, Vang):
        # brings angles to less than +/- max_degrees
        # max_degrees = 300.
        max_degrees = 180. #this will bring angles to within +/- 180 degrees
        Vang_wrap = Vang
        # if abs(Vang) > np.radians(max_degrees):
        #     if Vang > 0:
        #         Vang_wrap = Vang - np.radians(360.)
        #     elif Vang < 0:
        #         Vang_wrap = Vang + np.radians(360.)
        while abs(Vang_wrap) > np.radians(max_degrees):
            if Vang_wrap > 0:
                # print(f'Vang_wrap[{phase}] = {Vang_wrap[phase]}')
                Vang_wrap = Vang_wrap - np.radians(360.)
                print(f'SUBTRACTING 2pi radians in PhasorV_ang_wraparound from phase {phase} to get {Vang_wrap[phase]}')
            elif Vang_wrap < 0:
                # print(f'Vang_wrap[{phase}] = {Vang_wrap[phase]}')
                Vang_wrap = Vang_wrap + np.radians(360.)
                print(f'ADDING 2pi radians in PhasorV_ang_wraparound from phase {phase} to get {Vang_wrap[phase]}')
        return Vang_wrap

    def PhasorV_ang_wraparound(self, Vang, nphases, nameVang='(notgiven)'):
        # brings angles to less than +/- max_degrees
        # max_degrees = 300.
        max_degrees = 180. #this will bring angles to within +/- 180 degrees
        Vang_wrap = Vang
        for phase in range(nphases):
            # if abs(Vang[phase]) > np.radians(max_degrees):
            #     if Vang[phase] > 0:
            #         print(f'Vang[{phase}] = {Vang[phase]}')
            #         Vang_wrap[phase] = Vang[phase] - np.radians(360.)
            #         print(f'SUBTRACTING 2pi radians in PhasorV_ang_wraparound from {nameVang} phase {phase} to get {Vang_wrap[phase]}')
            #         # print(f'SUBTRACTING 2pi radians in PhasorV_ang_wraparound from phase {phase} to get {Vang_wrap[phase]}')
            #     elif Vang[phase] < 0:
            #         print(f'Vang[{phase}] = {Vang[phase]}')
            #         Vang_wrap[phase] = Vang[phase] + np.radians(360.)
            #         print(f'ADDING 2pi radians in PhasorV_ang_wraparound from {nameVang} phase {phase} to get {Vang_wrap[phase]}')
            #         # print(f'ADDING 2pi radians in PhasorV_ang_wraparound from phase {phase} to get {Vang_wrap[phase]}')
            while abs(Vang_wrap[phase]) > np.radians(max_degrees):
                if Vang_wrap[phase] > 0:
                    print(f'Vang_wrap[{phase}] = {Vang_wrap[phase]}')
                    Vang_wrap[phase] = Vang_wrap[phase] - np.radians(360.)
                    print(f'SUBTRACTING 2pi radians in PhasorV_ang_wraparound from {nameVang} phase {phase} to get {Vang_wrap[phase]}')
                    # print(f'SUBTRACTING 2pi radians in PhasorV_ang_wraparound from phase {phase} to get {Vang_wrap[phase]}')
                elif Vang_wrap[phase] < 0:
                    print(f'Vang_wrap[{phase}] = {Vang_wrap[phase]}')
                    Vang_wrap[phase] = Vang_wrap[phase] + np.radians(360.)
                    print(f'ADDING 2pi radians in PhasorV_ang_wraparound from {nameVang} phase {phase} to get {Vang_wrap[phase]}')
                    # print(f'ADDING 2pi radians in PhasorV_ang_wraparound from phase {phase} to get {Vang_wrap[phase]}')
        return Vang_wrap

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
        '''
        print('REF upmu0: ')
        print(reference_phasors[0][0])
        print(reference_phasors[1][0])
        print(reference_phasors[2][0])
        print('upmu4 voltage: ')
        print('PHASE A: ',local_phasors[0][0])
        print('PHASE B: ',local_phasors[1][0])
        print('PHASE C: ', local_phasors[2][0])
        print('current: ')
        print('PHASE A: ',local_phasors[3][0])
        print('PHASE B: ',local_phasors[4][0])
        print('PHASE C: ', local_phasors[5][0])
        '''

        iterstart = pytime.time()
        self.iteration_counter += 1
        print('iteration counter bus ' + str(self.busId) + ' : ' + str(self.iteration_counter))
        if self.iteration_counter > 1:
            print(f'time since last iteration {iterstart-self.iterstart}')
        self.iterstart = pytime.time()

        #Initilizes actuators, makes sure you're getting through to them
        if self.iteration_counter == 1:
            pass
            #HHERE commented out for debugging
            #could call CIL_debug.py (or a function that does what CIL_debug.py does) here to reset the Opal registers
            # (responseInverters, responseLoads) = self.initializeActuators(self.mode) #throws an error if initialization fails

        if phasor_target is None and self.VangTarg_relative == 'initialize':
            print('No target received from SPBC by bus ' + str(self.busId))
            return #don't need to return a status, when there isnt one to report
        else:
            if phasor_target is None:
                print('No target received by SPBC: Using last received target ' + str(self.busId))
            else:
                #get targets and bases from phasor_target, sent by the SPBC
                #values are ordered as: A,B,C according to availability, using the names given to the targets (by the SPBC)
                #VmagTarg is given as VmagTarg_relative_pu from the SPBC
                #5/28/20 SPBC (only) sends relative magnitude and angle targets (relative to the nominal reference, though SPBC does get the actual ref voltage, so that could be used later)
                #and apparently the relative voltages come in pu
                (self.VmagTarg_relative_pu, self.VangTarg_relative, self.kVbase, self.network_kVAbase, self.status_phases) = self.targetExtraction(phasor_target)
                print('VmagTarg_relative_pu bus ' + str(self.busId) + ' : ' + str(self.VmagTarg_relative_pu))
                print('VangTarg_relative bus ' + str(self.busId) + ' : ' + str(self.VangTarg_relative))
                print('kVbase bus ' + str(self.busId) + ' : ' + str(self.kVbase))
                print('network_kVAbase bus ' + str(self.busId) + ' : ' + str(self.network_kVAbase))
                # print('calculated Zbase bus ' + str(self.busId) + ' : ' + str(1000*self.kVbase*self.kVbase/self.network_kVAbase))
                print('status_phases bus ' + str(self.busId) + ' : ' + str(self.status_phases))
                self.kVbase  = np.asarray(self.kVbase)
                self.network_kVAbase = np.asarray(self.network_kVAbase)
                #phasor_target is (perLPBC) data packet from SPBC that contains channels (will be phases once fixed), V, delta, kvbase and kvabase
                self.localkVbase = self.kVbase/self.localVratio
                self.localkVAbase = self.network_kVAbase/self.localSratio #self.localkVAbase takes into account self.localSratio here
                self.localIbase = self.localkVAbase/self.localkVbase
                print('self.localSratio : ' + str(self.localSratio))
                print('self.localkVAbase : ' + str(self.localkVAbase))
                print('self.localkVbase : ' + str(self.localkVbase))

                if self.usingNonpuZeff and self.ZeffkestinitHasNotBeenInitialized:
                    Zbase = 1000*self.kVbase*self.kVbase/self.network_kVAbase #setup.py uses subkVbase_phg*subkVbase_phg*1000/subkVAbase to calc Zbase, so this is correct
                    print(f'SETTING Zeffkestinit with Zbase ({Zbase}) calculated using network_kVAbase ({self.network_kVAbase}) received from SPBC')
                    Zeffkestinit, self.ZeffkTru = self.controller.setZeffandZeffkestinitWnewZbase(Zbase, self.Zeffk_init_mult)
                    self.ZeffkestinitHasNotBeenInitialized = 0

            # calculate relative voltage phasor
            #the correct PMUs for voltage and current (ie uPMUP123 and uPMU123) are linked in the configuration phase, so local_phasors are what you want (already)
            #values are ordered as: A,B,C according to availability, using self.plug_to_phase_map
            # (self.Vang_notRelative,self.VangRef,self.Vang_relative,self.Vmag,self.VmagRef,self.Vmag_relative, local_time_index, ref_time_index, V_ang_ref_firstPhase, dataWindowLength, Vmeas_all_phases) = self.phasorV_calc(local_phasors, reference_phasors, self.nphases, self.plug_to_V_idx)
            (self.Vang_notRelative,self.VangRef,self.Vang_relative,self.Vmag,self.VmagRef,self.Vmag_relative, V_ang_ref_firstPhase, dataWindowLength, Vmeas_all_phases) = self.phasorV_calc(local_phasors, reference_phasors, self.nphases, self.plug_to_V_idx)
            # if any(np.isnan(self.Vang_relative)):
            if Vmeas_all_phases == 0:
                # print('Every phase has not received a relative phasor measurement yet, bus ' + str(self.busId))
                print(f'~~~ Didnt receive a measurement for each phase of bus {self.busId}, not running the controller this round. ~~~')
                return
            #these are used by the LQR controller
            self.Vang_notRelative = self.PhasorV_ang_wraparound(self.Vang_notRelative, self.nphases, nameVang='self.Vang_notRelative')
            self.Vang_relative = self.PhasorV_ang_wraparound(self.Vang_relative, self.nphases, nameVang='self.Vang_relative')
            self.VangRef = self.PhasorV_ang_wraparound(self.VangRef, self.nphases, nameVang='self.VangRef')
            self.Vmag_pu = self.Vmag / (self.localkVbase * 1000) # absolute
            self.Vmag_relative_pu = self.Vmag_relative / (self.localkVbase * 1000) #this and the VmagTarg_relative_pu line divides Vmag_ref by self.localkVbase which may create an issue bc Vref != 1.0pu, but thats okay
            self.VmagRef_pu = self.VmagRef / (self.localkVbase * 1000)
            self.VangTarg_notRelative = self.VangTarg_relative + self.VangRef
            self.VmagTarg_pu = self.VmagTarg_relative_pu + self.VmagRef_pu #VmagTarg is given as VmagTarg_relative_pu rn from the SPBC
            print('Vmag_pu bus ' + str(self.busId) + ' : ' + str(self.Vmag_pu))
            print('VmagRef_pu bus ' + str(self.busId) + ' : ' + str(self.VmagRef_pu))
            print('VmagTarg_pu bus ' + str(self.busId) + ' : ' + str(self.VmagTarg_pu))
            print('Vmag_relative_pu bus ' + str(self.busId) + ' : ' + str(self.Vmag_relative_pu))
            print('Vang_relative bus ' + str(self.busId) + ' : ' + str(self.Vang_relative))
            print('VangRef bus ' + str(self.busId) + ' : ' + str(self.VangRef))
            print('Vang_notRelative bus ' + str(self.busId) + ' : ' + str(self.Vang_notRelative))
            print('self.VangTarg_relative bus ' + str(self.busId) + ' : ' + str(self.VangTarg_relative))
            print('self.VangTarg_notRelative bus ' + str(self.busId) + ' : ' + str(self.VangTarg_notRelative))
            #this is here so that Relative angles can be used as LQR inputs (but with a non-relative Vcomp)
            Vcomp_pu = self.Vmag_pu*np.cos(self.Vang_notRelative) + self.Vmag_pu*np.sin(self.Vang_notRelative)*1j

            #these are used for PI controller
            self.phasor_error_ang = self.VangTarg_relative - self.Vang_relative
            self.phasor_error_mag_pu = self.VmagTarg_relative_pu - self.Vmag_relative_pu

            #get current measurements, determine saturation if current measurements exist
            if self.currentMeasExists:
                if len(local_phasors) >= self.nphases*2:
                    if self.controllerType == 'LQR':
                        #check if V_ang_ref_firstPhase and time_indexes exist
                        if any(np.isnan(V_ang_ref_firstPhase)) or any(np.isnan(ref_time_index)) or any(np.isnan(local_time_index)):
                            print('WARNING got a NaN V_ang_ref_firstPhase or ref_time_index or local_time_index')
                            self.Icomp_pu = [np.NaN]*self.nphases
                        else:
                            # self.Iang_notRelative, self.Imag = self.phasorI_calc(local_time_index, ref_time_index, V_ang_ref_firstPhase, dataWindowLength, local_phasors, self.nphases, self.plug_to_V_idx)
                            self.Iang_notRelative, self.Imag = self.phasorI_calc(dataWindowLength, local_phasors, reference_phasors, self.nphases, self.plug_to_V_idx)
                            self.Icomp = self.Imag*np.cos(self.Iang_notRelative) + self.Imag*np.sin(self.Iang_notRelative)*1j #shoudlnt need to unwrap currents
                            #HERE havent checked that the current measurements are legit yet
                            self.Icomp_pu = self.Icomp / self.localIbase #self.localIbase takes into account Sratio
                            #see comment above where Zeff is made. dividing by self.localIbase is eq to dividing by networkIbase then multuplying by Sratio, which gives the correct pu current for estimating the correct PU Zeff (that relates pu power injections to pu voltage)
                            self.Icomp_pu = -self.Icomp_pu #HERE in Flexlab current is positive flowing out of the Network, but want current to be positive into the network for Z estimation
                    # calculate P/Q from actuators
                    (self.Pact, self.Qact) = self.PQ_solver(local_phasors, self.nphases,self.plug_to_V_idx)  #HHERE 5/28/20: is this an issue: this is positive out of the network, which is backwards of Pcmd and Qcmd, bc thats how PMU 123 is set up in the flexlab
                    self.Pact_pu = self.Pact / self.localkVAbase
                    self.Qact_pu = self.Qact / self.localkVAbase

                    self.sat_arrayP, self.sat_arrayQ = self.checkSaturation(self.nphases, self.Pact, self.Qact, self.Pcmd_kVA, self.Qcmd_kVA, self.P_PV)  # returns vectors that are one where unsaturated and zero where saturated, will be unsaturated with initial Pcmd = Qcmd = 0
                else:
                    print('WARNING: self.currentMeasExists set to 1, but no/not enough current measurements given')
                    self.Icomp_pu = [np.NaN]*self.nphases
            else:
                self.Icomp_pu = [np.NaN]*self.nphases
                #havent built checkSaturationWoImeas yet
                self.sat_arrayP, self.sat_arrayQ = self.checkSaturationWoImeas(self.nphases, Vcomp_pu, self.Pcmd_kVA, self.Qcmd_kVA)

            #HERE [CHANGED] sign negations on Pact and Qact bc of dicrepancy between Pact convention and Pcmd convention
            (self.ICDI_sigP, self.ICDI_sigQ, self.Pmax_pu, self.Qmax_pu) = self.determineICDI(self.nphases, self.sat_arrayP, self.sat_arrayQ, -self.Pact_pu, -self.Qact_pu) #this and the line above have hardcoded variables for Flexlab tests

            #run control loop
            #print('self.phasor_error_mag_pu ' + str(self.phasor_error_mag_pu))
            #print('self.phasor_error_ang ' + str(self.phasor_error_ang))
            print('self.sat_arrayP ' + str(self.sat_arrayP))
            print('self.sat_arrayQ ' + str(self.sat_arrayQ))
            if self.controllerType == 'PI':
                if any(np.isnan(self.phasor_error_mag_pu)) or any(np.isnan(self.phasor_error_ang)):
                    print('GOT A NAN ERROR, USING P AND Q COMMANDS FROM PREVIOUS STEP')
                    controlStepTaken = 0
                else:
                    controlStepTaken = 1
                    (self.Pcmd_pu,self.Qcmd_pu) = self.controller.PIiteration(self.nphases,self.phasor_error_mag_pu, self.phasor_error_ang, self.sat_arrayP, self.sat_arrayQ)
            elif self.controllerType == 'LQR':
                #below lines could use self.Vang_relative and self.VangTarg_relative instead, if controller is being run with relative measurements
                if any(np.isnan(self.Vmag_pu)) or any(np.isnan(self.Vang_notRelative)) or any(np.isnan(self.VmagRef_pu)) or any(np.isnan(self.VangRef)): #used when phasorV_calc makes default measurements NaN rather than previous meas
                    print('GOT A NAN MEAS, USING P AND Q COMMANDS FROM PREVIOUS STEP')
                    controlStepTaken = 0
                elif any(np.isnan(self.VmagTarg_pu)) or any(np.isnan(self.VangTarg_notRelative)):
                    print('GOT A NAN TARG, USING P AND Q COMMANDS FROM PREVIOUS STEP')
                    controlStepTaken = 0
                else:
                    controlStepTaken = 1
                    if self.currentMeasExists:
                        if self.useRelativeMeas: #default is 0. setting to 1 runs LQR with relative V measurements rather than nonRelative V measurements (still uses relative Vcomp)
                            # need self.linearizeplant = 1 within LQR eqns, which is the default I think
                            fakeVangRef = np.zeros(self.nphases)
                            self.Pcmd_pu, self.Qcmd_pu, Zeffkest, Gt = self.controller.LQRupdate(self.Vmag_pu, self.Vang_relative, self.VmagTarg_pu, self.VangTarg_relative, self.VmagRef_pu, fakeVangRef, self.P_implemented_PU, self.Q_implemented_PU, self.sat_arrayP, self.sat_arrayQ, IcompArray=self.Icomp_pu, VcompArray=Vcomp_pu) #Vcomp_pu is still not relative, so the Zestimator can work
                        else:
                            self.Pcmd_pu, self.Qcmd_pu, Zeffkest, Gt = self.controller.LQRupdate(self.Vmag_pu, self.Vang_notRelative, self.VmagTarg_pu, self.VangTarg_notRelative, self.VmagRef_pu, self.VangRef, self.P_implemented_PU, self.Q_implemented_PU, self.sat_arrayP, self.sat_arrayQ, IcompArray=self.Icomp_pu) #all Vangs must be in radians
                    else:
                        if self.useRelativeMeas:
                            self.Pcmd_pu,self.Qcmd_pu, Zeffkest, Gt = self.controller.LQRupdate(self.Vmag_pu, self.Vang_relative, self.VmagTarg_pu, self.VangTarg_relative, self.VmagRef_pu, fakeVangRef, self.P_implemented_PU, self.Q_implemented_PU, self.sat_arrayP, self.sat_arrayQ, VcompArray=Vcomp_pu) #Vcomp_pu is still not relative, so the Zestimator can work
                        else:
                            self.Pcmd_pu,self.Qcmd_pu, Zeffkest, Gt = self.controller.LQRupdate(self.Vmag_pu, self.Vang_notRelative, self.VmagTarg_pu, self.VangTarg_notRelative, self.VmagRef_pu, self.VangRef, self.P_implemented_PU, self.Q_implemented_PU, self.sat_arrayP, self.sat_arrayQ)

            print('Pcmd_pu bus ' + str(self.busId) + ' : ' + str(self.Pcmd_pu))
            print('Qcmd_pu bus ' + str(self.busId) + ' : ' + str(self.Qcmd_pu))
            print('localkVAbase bus ' + str(self.busId) + ' : ' + str(self.localkVAbase))

            print('Zeffkest bus ' + str(self.busId) + ' : ' + str(Zeffkest))
            print('ZeffkestErr bus ' + str(self.busId) + ' : ' + str(np.linalg.norm(Zeffkest-self.ZeffkTru)))
            print('GtMag ' + str(self.busId) + ' : ' + str(np.linalg.norm(Gt)))
            #in case you want to save and plot the Zeff error:
            # self.ZeffkError.append(np.linalg.norm(Zeffkest-self.ZeffkTru)) #frob norm is default
            # self.GtMag.append(np.linalg.norm(Gt))

            # The controller is entirely in PU. So if the pu P and Q commands are ultimately enacted on the network
            #according to the network-wide kVAbase (that was used to calculate the Zbase that was used to build the controller), then there shouldn’t be any problems.
            # The Sratio multiplication should not mess up the I estimation from S command (for Z estimation) bc the Sratio multiplication is canceled out when the Base is divided by the Sratio.
            #(Zest should use self.network_kVAbase not self.localkVAbase)

            #HHHERE for debugging
            # self.Pcmd_pu = np.zeros(3)
            # self.Qcmd_pu = np.zeros(3)
            # self.Pcmd_pu = np.ones(3)*.2
            # self.Qcmd_pu = np.ones(3)*.2

            if self.perturbPowerCommand:
                self.Pcmd_pu = self.Pcmd_pu + np.random.randn(self.nphases) * self.perturbScale
                self.Qcmd_pu = self.Qcmd_pu + np.random.randn(self.nphases) * self.perturbScale

            # self.localkVAbase = self.network_kVAbase/self.localSratio, so this assumes that the power injections will later get multiplied by self.localSratio
            self.Pcmd_kVA = self.Pcmd_pu * self.localkVAbase #these are positive for power injections, not extractions
            self.Qcmd_kVA = self.Qcmd_pu * self.localkVAbase #localkVAbase takes into account that network_kVAbase is scaled down by localSratio (divides by localSratio)
            #localkVAbase is not a good name (bc its not the same thing as how voltage bases change throughout a network)
            #instead localkVAbase should be called flexlabAdjustedkVAbase #HHERE

            if self.actType == 'inverter':
                if self.currentMeasExists or self.mode == 3 or self.mode == 4 or True: #HHHERE put in the or True when I set the self.currentMeasExists to 0 manually
                    '''
                    COMMENTED OUT FOR CIL TESTING
                    # self.commandReceipt, self.P_implemented_PU, self.Q_implemented_PU = self.httptoInverters(self.nphases, self.act_idxs, self.Pcmd_kVA, self.Qcmd_kVA, self.Pact) #calculating Pact requires an active current measurement
                    self.commandReceipt = self.httptoInverters(self.nphases, self.act_idxs, self.Pcmd_kVA, self.Qcmd_kVA, self.Pact) #calculating Pact requires an active current measurement
                    self.P_implemented_PU = self.Pcmd_pu #HERE change these if inverter commands are not always realized
                    self.Q_implemented_PU = self.Qcmd_pu
                    print('inverter command receipt bus ' + str(self.busId) + ' : ' + str(self.commandReceipt))
                    '''
                    print('********')
                    print('Vmag_relative_pu bus ' + str(self.busId) + ' : ' + str(self.Vmag_relative_pu))
                    print('Vang bus ' + str(self.busId) + ' : ' + str(self.Vang_relative))
                    print('self.phasor_error_mag_pu ' + str(self.phasor_error_mag_pu))
                    print('self.phasor_error_ang ' + str(self.phasor_error_ang))
                    result, self.P_implemented_PU, self.Q_implemented_PU = self.modbustoOpal(self.nphases, self.Pcmd_kVA, self.Qcmd_kVA, self.ORT_max_VA,self.localSratio, self.client)
                    # result = self.modbustoOpal(self.nphases, self.Pcmd_kVA, self.Qcmd_kVA, self.ORT_max_VA,self.localSratio, self.client)
                    print('Opal command receipt bus ' + str(self.busId) + ' : ' + str(result))
                else:
                    print('couldnt send commands because no current measurement available') #HERE what?
            elif self.actType == 'load':
                # self.commandReceipt, self.P_implemented_PU, self.Q_implemented_PU = self.httptoLoads(self.nphases, self.act_idxs, self.Pcmd_kVA, self.Qcmd_kVA)
                self.commandReceipt = self.httptoLoads(self.nphases, self.act_idxs, self.Pcmd_kVA, self.Qcmd_kVA)
                self.P_implemented_PU = self.Pcmd_pu #HERE change these if load commands are not always realized
                self.Q_implemented_PU = self.Qcmd_pu
                print('load command receipt bus ' + str(self.busId) + ' : ' + str(self.commandReceipt))
            elif self.actType == 'modbus':
                result, self.P_implemented_PU, self.Q_implemented_PU = self.modbustoOpal(self.nphases, self.Pcmd_kVA, self.Qcmd_kVA, self.ORT_max_VA, self.localSratio)
                # result = self.modbustoOpal(self.nphases, self.Pcmd_kVA, self.Qcmd_kVA, self.ORT_max_VA, self.localSratio)
                print('Opal command receipt bus ' + str(self.busId) + ' : ' + str(result))
            else:
                error('actType error')

            # #Hack to get self.P_implemented_PU and self.Q_implemented_PU (assumes max_kVA is implemented correctly by self.modbustoOpal, self.httptoLoads or self.httptoInverters + self.modbustoOpal_quadrant combo)
            # max_PU_power = self.ORT_max_VA/1000/self.network_kVAbase #HHERE
            # if self.Pcmd_pu > max_PU_power: # P and Q commands get compared with max_kVA indepenedently
            #     used_Pcmd_pu = max_PU_power
            # elif self.Pcmd_pu < -max_PU_power:
            #     used_Pcmd_pu = -max_PU_power
            # else:
            #     used_Pcmd_pu = self.Pcmd_pu
            # if self.Qcmd_pu > max_PU_power: # P and Q commands get compared with max_kVA indepenedently
            #     used_Qcmd_pu = max_PU_power
            # elif self.Qcmd_pu < -max_PU_power:
            #     used_Qcmd_pu = -max_PU_power
            # else:
            #     used_Qcmd_pu = self.Qcmd_pu
            # self.P_implemented_PU = used_Pcmd_pu
            # self.Q_implemented_PU = used_Qcmd_pu
            print('self.P_implemented_PU ', self.P_implemented_PU)
            print('self.Q_implemented_PU ', self.Q_implemented_PU)
            #HERE self.P_implemented_PU could be self.Pact_PU, but self.Pact_PU requires a PMU current meas, so have to use an if statement to set self.P_implemented_PU with P_act
            #(could get rid of self.P_implemented_PU and just keep self.Pact_PU)

            self.Pact_kVA = self.Pact
            self.Qact_kVA = self.Qact

            #HHERE need to adjust these so that they log self.P_implemented_PU and self.Q_implemented_PU too
            log_actuation = self.save_actuation_data(self.status_phases, self.Pcmd_kVA, self.Qcmd_kVA, self.Pact_kVA, self.Qact_kVA, self.P_PV, self.batt_cmd, self.pf_ctrl)
            self.log_actuation(log_actuation)
            # print(log_actuation)
            status = self.statusforSPBC(self.status_phases, self.phasor_error_mag_pu, self.phasor_error_ang, self.ICDI_sigP, self.ICDI_sigQ, self.Pmax_pu, self.Qmax_pu)
            # print(status)
            iterend = pytime.time()

            print(f'~~~ STEP FINISH - iter length: {iterend-iterstart}, epoch: {pytime.time()} ~~~')
            print('')
            if (iterend-iterstart) > rate:
                print('WARNING: LOOP LENGTH LARGER THAN RATE - INCREASE SIZE OF RATE')
                print('')

            #trying to mimic lpbcwrapper env for last lpbc in lpbcdict
            # iter = self.iteration_counter - 1
            iter = self.controlStepsTaken_counter
            if controlStepTaken == 1:
                self.controlStepsTaken_counter += 1
                print('self.controlStepsTaken_counter ', self.controlStepsTaken_counter)
                if iter < self.HistLength:
                    if self.controllerType == 'LQR':
                        self.ZeffkErrorHist[iter] = np.linalg.norm(Zeffkest-self.ZeffkTru) #frob norm is default
                        self.GtMagHist[iter] = np.linalg.norm(Gt)
                    self.VmagHist[:,iter] = self.Vmag_pu
                    self.VangHist[:,iter] = self.Vang_relative
                    # self.ZeffkErrorHist.append(np.linalg.norm(Zeffkest-self.ZeffkTru)) #frob norm is default
                    # self.GtMagHist.append(np.linalg.norm(Gt))
                    print('SAVING measurements for plotting')
                elif iter == self.HistLength:
                    print('$$$$$$$$$$$$$$$$$$$$$$ SAVING plots $$$$$$$$$$$$$$$$$$$$$$')
                    if self.saveVmagandangPlot or self.saveZesterrorPlot:
                        current_directory = os.getcwd()
                        resultsPATH = os.path.join(current_directory, 'simulationPlots')
                        resultsPATH = os.path.join(resultsPATH, f'feeder:{self.testcase}_bus:{self.busId}') #if you want to write over previous run
                        # resultsPATH = os.path.join(resultsPATH, f'feeder:{self.testcase}_bus:{self.busId}_time:{pytime.time()}') #if you want to save each run
                        if not os.path.exists(resultsPATH):
                            os.makedirs(resultsPATH)

                    if self.saveVmagandangPlot:
                        #magnitude
                        for phase in np.arange(self.controller.nphases):
                            plt.plot(self.VmagHist[phase,:], label='node: ' + self.busId + ', ph: ' + str(phase))
                        print('phase ', phase)
                        print('self.VmagTarg_pu[phase] ', self.VmagTarg_pu[phase])
                        plt.plot(self.VmagTarg_pu[phase]*np.ones(self.HistLength),'-', label='node: ' + key + ', target')
                        # plt.title('Network: 13 node feeder with constant load')
                        plt.ylabel('p.u. Vmag')
                        plt.xlabel('Timestep')
                        plt.legend()
                        plt.savefig(os.path.join(resultsPATH, 'Vmag')); plt.clf(); plt.cla(); plt.close()

                        #angle
                        print('self.VangHist ', self.VangHist)
                        for phase in np.arange(self.controller.nphases):
                            Vangs = self.VangHist[phase,:]
                            plt.plot(Vangs, label='node: ' + key + ', ph: ' + str(phase))
                        plt.plot(self.VangTarg_relative[0]*np.ones(self.HistLength),'-', label='node: ' + key + ', phase A target')
                        # plt.title('Network: 13 node feeder with constant load')
                        plt.ylabel('Vang [rad]')
                        plt.xlabel('Timestep')
                        plt.legend()
                        plt.savefig(os.path.join(resultsPATH, 'Vang')); plt.clf(); plt.cla(); plt.close()
                        print('^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^')
                        print('SAVED Vmag and Vang plots ')

                    if self.saveZesterrorPlot and self.controllerType == 'LQR':
                        Zeffkinit = self.ZeffkTru*self.Zeffk_init_mult
                        print(f'Zeffk_true (PU) bus {self.busId}: ', self.ZeffkTru)
                        print(f'Zeffk_init (PU) bus {self.busId}: ', Zeffkinit)
                        print(f'Zeffk_est (PU) bus {self.busId}: ', Zeffkest)
                        plt.plot(self.ZeffkErrorHist,'-', label='node: ' + key)
                        # plt.title('Zeff Estimation Error')
                        plt.ylabel('Frobenius Norm Zeff Estimation Error')
                        plt.xlabel('Timestep')
                        plt.legend()
                        plt.savefig(os.path.join(resultsPATH, 'ZestError')); plt.clf(); plt.cla(); plt.close()

                        plt.plot(self.GtMagHist,'-', label='node: ' + key)
                        plt.ylabel('Frobenius Norm of Gt')
                        plt.xlabel('Timestep')
                        plt.legend()
                        plt.savefig(os.path.join(resultsPATH, 'Gt')); plt.clf(); plt.cla(); plt.close()
                        print('^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^')
                        print('SAVED Zest plots ')

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
#testcase = '13unb'
#testcase = '13bal'
# testcase = '33'
testcase = 'PL0001'
# testcase = 'manual'

acts_to_phase_dict = dict()
actType_dict = dict()
if testcase == '13bal':
    testNumber = '3.3'
    lpbcidx = ['675'] #may have to set these manually
    for key in lpbcidx: #makes them all three phase inverters
        acts_to_phase_dict[key] = np.asarray(['A','B','C']) #3 phase default #['A','',''] or ['','C',''] or ['A','B','C','A','B','C'] or ['A','','','A','',''] are also examples, ['A','C','B'] and ['B','B','B'] are not allowed (yet)
        actType_dict[key] = 'inverter' #'inverter' or 'load'
    ORT_max_kVA = 500
    VmagScaling = 1
    inverterScaling = 500/3.3
    loadScaling = 350
    CILscaling = 10 #in VA
elif testcase == '13unb':
    # lpbcidx = ['671','680']
    # key = '671'
    # acts_to_phase_dict[key] = np.asarray(['A','B','C']) #phase on the network (in simulation)
    # actType_dict[key] = 'inverter'
    # key = '680'
    # acts_to_phase_dict[key] = np.asarray(['','','C']) #HERE Single phase actuation might cause problems #the nonzero entries correspond to the actuator indices
    # actType_dict[key] = 'load'
    testNumber = '8.1'
    lpbcidx = ['632'] #nodes of actuation
    key = '632'
    acts_to_phase_dict[key] = np.asarray(['A','B','C']) #which phases to actuate for each lpbcidx # INPUT PHASES
    actType_dict[key] = 'inverter' #choose: 'inverter', 'load', or 'modbus'
    ORT_max_kVA = 500
    VmagScaling = 1
    inverterScaling = 500/3.3
    loadScaling = 350
    CILscaling = 10 #in VA
elif testcase == '33':
    testNumber = '8.1'
    lpbcidx = ['6'] #for 33
    key = '6'
    acts_to_phase_dict[key] = np.asarray(['A','B','C']) #which phases to actuate for each lpbcidx # INPUT PHASES
    actType_dict[key] = 'inverter' #choose: 'inverter', 'load', or 'modbus'
    ORT_max_kVA = 500
    VmagScaling = 3. #this is a hack to get flexlab to work
    inverterScaling = 500/3.3
    loadScaling = 350
    CILscaling = 10 #in VA
elif testcase == 'PL0001':
    testNumber = '9.3'
    lpbcidx = ['N_300063911']
    key = 'N_300063911'
    acts_to_phase_dict[key] = np.asarray(['A','B','C']) #which phases to actuate for each lpbcidx # INPUT PHASES
    actType_dict[key] = 'inverter' #choose: 'inverter', 'load', or 'modbus'
    ORT_max_kVA = 1000
    VmagScaling = 12.6/4.16
    inverterScaling = 1000/1
    loadScaling = 350
    CILscaling = 20 #in VA
elif testcase == 'manual':
    print('MOVED TESTS TO ACTUAL TEST CASES')


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
    if 'A' in acts_to_phase_dict[key]: #HERE if you wanted to always include the ref meas for phase A you would take out this if statement. Would also require changing the logic in phasorV_calc and phasorI_calc.
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
'''NOTE: CHANGED PMUS TO CONFIGURE TO CIL TESTING BECAUSE COULD NOT FIGURE OUT HOW TO GET THE PMUS WITHOUT ERROR'''
#pmu123Channels = np.asarray(['uPMU_123/L1','uPMU_123/L2','uPMU_123/L3','uPMU_4/C1','uPMU_4/C2','uPMU_4/C3'])
pmu123Channels = np.asarray([]) # DONE FOR CIL

#HHERE this is a hack that Leo implemented to avoid the plug mapping
if testNumber == '3.3' or testNumber == '9.3':
    pmu123PChannels = np.asarray(['uPMU_123P/L1','uPMU_123P/L2','uPMU_123P/L3'])
elif testNumber == '8.1':
    pmu123PChannels = np.asarray(['uPMU_4/L1','uPMU_4/L2','uPMU_4/L3']) #these also have current channels, but dont need them
else:
    error('Not a valid test number yet (determine what pmu123PChannels should be and include it in the if statements)')

pmu4Channels = np.asarray(['uPMU_4/L1','uPMU_4/L2','uPMU_4/L3'])
refChannels = np.asarray(['uPMU_0/L1','uPMU_0/L2','uPMU_0/L3','uPMU_0/C1','uPMU_0/C2','uPMU_0/C3'])

nlpbc = len(lpbcidx)

#cfg file is used to build each LPBC, this is a template that is modified below for each LPBC
cfg_file_template = config_from_file('template.toml') #config_from_file defined in XBOSProcess

#CILscaling = Sratio (below):
# command given will get multiplied by [150] in switch matrix
# then divided by 15,000 to give a value in kW internally in Flexlab OpalRT (I think)
# Thus a VA command will be multiplied by 10 (10 = 150/15 = 150/(15000/1000))
# Sratio divides the network kVA
# Sratio=10 divides the networkkVAbase by 10, so when the PU power commands are multiplied by kVA base they will implicitly be divided by 10, which cancels out the factor of 10 that the switch matrix scaling contributes.

# rate = 5
rate = 10 #HHHERE for debugging issue with timestep 1 command

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
        cfg['local_channels'] = list(pmu123PChannels[pmu123P_plugs_dict[key]])
        #COMMENTED LINE BELOW FOR CIL TESTING
        #cfg['local_channels'] = list(np.concatenate([pmu123PChannels[pmu123P_plugs_dict[key]], pmu123Channels[3 + pmu123_plugs_dict[key]], pmu123Channels[pmu123_plugs_dict[key]]]))
        #takes voltage measurements from PMU123P, current from PMU123, voltage measurements from PMU123P
        cfg['reference_channels'] = list(refChannels[pmu0_plugs_dict[key]]) #assumes current and voltage plugs are connected the same way
        currentMeasExists = True
        '''
        COMMENTED OUT FOR CIL TESTING
        localSratio = inverterScaling
        '''
        localSratio = CILscaling

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
    cfg['testcase'] = testcase #6/3/20 put this in so the wrapper plotter can use the name to save the plot for a given testcase
    # currentMeasExists = 0 #HHHERE delete this -- set to 0 in order to run Zest in CIL test
    lpbcdict[key] = lpbcwrapper(cfg, key, testcase, nphases, act_idxs, actType, plug_to_phase_idx, timesteplength, currentMeasExists, localSratio, ORT_max_kVA, VmagScaling) #Every LPBC will have its own step that it calls on its own
    #key is busId, which is the performance node for the LPBC (not necessarily the actuation node)

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
