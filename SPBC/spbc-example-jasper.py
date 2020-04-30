from pyxbos.process import run_loop, schedule, config_from_file
from pyxbos.drivers import pbc
import logging
import random
import datetime
import numpy as np

#import spbc related functions
from setup_3 import *
from constraints_3 import *
from dss_3 import *
from main_run_3 import *

logging.basicConfig(level="INFO", format='%(asctime)s - %(name)s - %(message)s')

#init feeder & other vars (commented below because excel file is incomplete)
#phase_size, feeder_init = feeder_init()
#print('phases on network:',phase_size)

# SETTINGS
lpbc_phases = ['a'] # [INPUT HERE]
lpbc_nodeIDs = ['18'] # [INPUT HERE]
angle_unit = 'radians' # - 'degrees' or 'radians' - settled on radians

TV_load = False # [INPUT HERE] - set whether SPBC cycles through load values or holds constant
start_hour = 11 # [INPUT HERE]

dummy_ref = True # [INPUT HERE]
constant_phasor = True # [INPUT HERE]

if dummy_ref == True:
    print('WARNING: constant_ref ON')
if constant_phasor == True:
    # set phasor target values here (not relative)
    #cons_Vmag = [0.9862920,0.9956446,0.9881567] # [INPUT HERE]
    # cons_Vmag - 1 = Vmag_relative_pu (where 1, is 1pu at ref/feeder head)
    cons_Vmag = [0.99,0.99,0.99] # [INPUT HERE]
    #cons_Vang = [-1.61526,-121.75103,118.20174]
    #cons_Vang = [0-1,-120-1,120-1] # [INPUT HERE]
    # USED BELOW ONLY FOR T12
    cons_Vang = [0 - 1, -120 - 1, 120 - 1]
    cons_kVbase = np.ones(3)*4.16/np.sqrt(3) # [INPUT HERE]
    cons_kVAbase = np.ones(3)*5000/3 # [INPUT HERE]
    print('WARNING: constant_phasor ON')


# TODO: vmagprev, check dims across all instances, think it shoud just be 3
Vmag_prev = []
#Vmag_prev = {}
#Vmag_prev[key] = np.ones((3))*np.inf

# TODO; put lpbc_nodes outsie of loop in init section?
#lpbc_nodes = []

print()
print('$$$$$$$$$$$$$$$$')
print('~~ START SPBC ~~')
print('$$$$$$$$$$$$$$$$')

class myspbc(pbc.SPBCProcess):
    """
    This is an example SPBC implementation demonstrating how to access and use
    the data available through XBOS.

    Configuration:
    - namespace: do not change
    - wavemq: address of local wavemq agent
    - name: name of the SPBC controller. Naming each SPBC allows us to have multiple
      parallel SPBCs in operation. LPBCs choose which SPBC they want to listen to by
      specifying this name in their own configuration.
    - entity: the name of the local file constituting the 'identity' of this process.
      The entity file is what gives this process the permission to interact with other
      resources
    - reference_channels: a list of URIs representing the phasor channels the SPBC
      subscribes to as reference phasors


    Data: the SPBC has several data streams available to it as a result of its configuration.
    - self.reference_phasors: contains the most recently received phasors from each of the reference
      phasor channels. This only contains the data from the most recently received phasor message,
      which *does not* consist of all the data since the SPBC "compute_and_announce" was last run
      The SPBC framework keeps the self.reference_phasors data up to date in the background.

      self.reference_phasors is a dictionary keyed by the names of the phasor channels indicated
      in the 'reference_channels' configuration option. For example, if the SPBC is configured
      with "reference_channels = ['flexlab1/L1','flexlab1/L2']", then self.reference_phasors would
      contain the following structure:

        self.reference_phasors = {
            'flexlab1/L1': [
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
                ... etc
            ],
            'flexlab/L2': [
                {
                    "time": "1559231114799996800",
                    "angle": 220.30149788923268,
                    "magnitude": 10.038565948605537415
                },
                {
                    "time": "1559231114899996400",
                    "angle": 220.50249902851263,
                    "magnitude": 10.042079225182533264
                }
            ]
        }
    - self.lpbcs: contains the most recent LPBC statuses. By default, the SPBC subscribes to
      all LPBC instances it has permission to access. The SPBC framework subscribes to the
      LPBC statuses in the background and transparently updates the self.lpbcs structure.
      self.lpbcs is a dictionary keyed by the names of each LPBC. Each value is another dictionary
      for that LPBC node, keyed by the channels for the LPBC (e.g. L1)

        self.lpbcs = {
            # node name
            'lpbc_1': {
                # phase names
                'L1': {
                    # local time of LPBC
                    'time': 1559231114799996800,
                    # phasor errors of LPBC
                    'phasor_errors': {
                        'angle': 1.12132,
                        'magnitude': 31.12093090,
                        # ... and/or ...
                        'P': 1.12132,
                        'Q': 31.12093090,
                    },
                    # true if P is saturated
                    'pSaturated': True,
                    # true if Q is saturated
                    'qSaturated': True,
                    # if p_saturated is True, expect the p max value
                    'pMax': 1.4,
                    # if q_saturated is True, expect the q max value
                    'qMax': 11.4,
                },
            },
            # etc...
        }
    """
    def __init__(self, cfg):
        super().__init__(cfg)
        # TODO: store Vmag of previous timestep for HIL implementation (only 1 timestep at a time...)

        # Create whatever instance variables + initialization you want here.
        # Pass options in using the 'cfg' dictionary
        
        # define system phase size to define size of phasor reference
        #phase_size, feeder_init = feeder_init() ## doesnt work inside of __init__

        
        # This particular implementation calls the self.compute_and_announce function
        # every 3 seconds; the self.compute_and_announce contains the optimization function
        # that produces the phasor target for each LPBC
        schedule(self.call_periodic(30, self.compute_and_announce))
        ### set some refphasor variable == true/false to determine length of schedule
        
         #~~ initialize values ~~#
        self.iteration = -1
        # self.P_flag = []
        # self.Q_flag = []
        if TV_load == True:
            self.timestepcur = start_hour*60-1
        else:
            self.timestepcur = start_hour*60
        
        ''' # add initializtion of LPBC nodes here, and then add update in loop
        for lpbc, channels in self.lpbcs.items():
            for channel, status in channels.items():    
                for key, ibus in feeder_init.busdict.items():
                            #if lpbc == 'lpbc_' + key:
                            if lpbc == key:
                                self.lpbc_nodes.append(key)
        # hardcode lpbc_nodes in
        self.lpbc_nodes = ['675'] # [INPUT HERE]
        '''

    async def compute_and_announce(self):
        print('')
        print('~~~ New compute_and_announce instance ~~~')
        
        #~~ initialize values ~~#
            
        self.iteration += 1
        if TV_load == True:
            self.timestepcur += 1
            
        print(f'iteration: {self.iteration}, timestep: {self.timestepcur-start_hour*60}')
        
        # ~~ LPBC ~~ #
        Psat_nodes = []
        Qsat_nodes = []
        lpbc_nodes = []
        
        # TODO: do lpbc's live at perf nodes not at all act nodes...
            # write method to send out voltage targets for perf nodes
            # for method below to work need lpbc to announce even if no targets to report 
                # (i.e. before SPBC sends any targets)
                # could also input perf nodes manually
                
        # how to loop through all LPBC statuses
        for lpbc, channels in self.lpbcs.items():
            for channel, status in channels.items():
                print('LPBC status:', lpbc,':', channel, ':', status)

                if channel == 'ph_A':
                    chanph = 'a'
                if channel == 'ph_B':
                    chanph = 'b'
                if channel == 'ph_C':
                    chanph = 'c'
                # get perf nodes (lpbc nodes)
                #for key, ibus in feeder_init.busdict.items(): #DEBUGGING
                for key in lpbc_nodeIDs:
                    #if lpbc == 'lpbc_' + key:
                    if lpbc == key:
                        lpbc_nodes.append(key)
        
        # create list of nodes where ICDI is true (Change to distinguish b/w P & Q)
                if status['pSaturated'] == True:
                    #Psat_nodes.append(lpbc[5:]+'_'+chanph)
                    Psat_nodes.append(lpbc + '_' + chanph)
                if status['qSaturated'] == True:
                    #Qsat_nodes.append(lpbc[5:]+'_'+chanph)
                    Qsat_nodes.append(lpbc + '_' + chanph)
                    
# =============================================================================
# =============================================================================
#                      if status['pSaturated'] == True:
#                          Psat_nodes.append(f'{lpbc[5:]}_a')
#                      if status['qSaturated'] == True:
#                         Qsat_nodes.append(lpbc[5:])
# =============================================================================
# =============================================================================
                    
        # hardcode lpbc_nodes in
        lpbc_nodes = lpbc_nodeIDs
        
        print('Psat',Psat_nodes)
        print('Qsat',Qsat_nodes)
        
        # ~~~~~~~~~~~~~~~~~~~~~~ #
        # ~~ REFERENCE PHASOR ~~ #
        # ~~~~~~~~~~~~~~~~~~~~~~ #
        
        # num of ref channels must match num of phases for slack bus on impedance model
         # otherwise read ref phasor error is returned
        phase_size = 3
        refphasor_init = np.ones((phase_size,2))*np.inf
        refphasor = refphasor_init
        # how to loop through all reference phasor channels
        print('here0')
        for channel, data in self.reference_phasors.items():
            print(f"Channel {channel} has {len(data) if data else 0} points")
            if data != None:
                #store most recent uPMU ref phasor values
                if 'L1' in channel:
                    refphasor[0,0] = data[-1]['magnitude']
                    refphasor[0,1] = data[-1]['angle']             
                if 'L2' in channel:
                    refphasor[1,0] = data[-1]['magnitude']
                    refphasor[1,1] = data[-1]['angle']
                if 'L3' in channel:
                    refphasor[2,0] = data[-1]['magnitude']
                    refphasor[2,1] = data[-1]['angle']
                
# =============================================================================
#                 try:
#                     pmutime = int(data[-1]['time'])
#                     print(f'pmu timestamp {pmutime}')
#                     pmu_s = pmutime / 1e9
#                     print(f'pmu latency: {time.time()-pmu_s}')
#                 except Exception as e:
#                     print(e)
# =============================================================================
        
            
        #convert Vmag to p.u. (subKVbase_phg defined in main)
        #commented below for debugging
        #refphasor[:,0] = refphasor[:,0]/(subkVbase_phg*1000) # TODO: compute refphasor vmag correctly
        #convert angle from degrees to rads
        # TODO: phases in right order?
        #[[1.00925961 2.04308987]
        #[1.00899569 6.2332654 ]
        #[1.01064548 4.13935041]]
        #refphasor[:,1] = refphasor[:,1]*np.pi/180 # TODO: change phB to -120 first?
        
        if dummy_ref == False:
            print('phasor reference [pu-rad]:')
            print(refphasor)

        if dummy_ref == True:
            #dummy values
            refphasor = refphasor_init
            refphasor[:,0]=1
            refphasor[:,1]=[0,4*np.pi/3,2*np.pi/3]
            print('using nominal reference values')
                         
        
        if np.inf in refphasor:
            ### WAIT TILL NEXT ###
            #dummy values
            #refphasor = refphasor_init
            #refphasor[:,0]=1
            #refphasor[:,1]=[0,4*np.pi/3,2*np.pi/3]
            print('Reference uPMU read error')
            
        else:
            # you could do expensive compute to get new targets here.
            # This could produce some intermediate structure like so:
            print('here') #pass here because excel file is incomplete
            #Vtargdict, act_keys, subkVAbase, myfeeder = spbc_run(refphasor,Psat_nodes,Qsat_nodes,lpbc_nodes,self.timestepcur)
            
            # TODO: how do we communicate phase information?
            # None-padded? dicts keyed by the channel name?
            # should set computed targets to have lpbc_nodeID so they dont have to be ordered specifically

            if constant_phasor == True:
                print('here2')
                Vtargdict = {}
                refphasor[:,1] = refphasor[:,1]*180/np.pi
                refphasor[1,1] = refphasor[1,1]-360
                for key in lpbc_nodes:
                    Vtargdict[key] = {}
                    Vtargdict[key]['Vmag'] = [cons_Vmag[0]-refphasor[0,0],cons_Vmag[1]-refphasor[1,0],cons_Vmag[2]-refphasor[2,0]]
                    Vtargdict[key]['Vang'] = [cons_Vang[0]-refphasor[0,1],cons_Vang[1]-refphasor[1,1],cons_Vang[2]-refphasor[2,1]]

                    # if self.iteration < 13*2:
                    '''CHANGED INDICIES FOR T12 CONFIG ONLY'''
                    # Vtargdict[key]['Vmag'] = [cons_Vmag[0]-refphasor[0,0],cons_Vmag[1]-refphasor[0,0],cons_Vmag[2]-refphasor[0,0]]
                    # Vtargdict[key]['Vang'] = [cons_Vang[0]-refphasor[0,1],cons_Vang[1]-refphasor[0,1],cons_Vang[2]-refphasor[0,1]]
                    # if self.iteration > 31:
                    #     Vtargdict[key]['Vmag'] = [0.98 - refphasor[0, 0], 0.98 - refphasor[1, 0],0.98 - refphasor[2, 0]]
                    #     Vtargdict[key]['Vang'] = [-2 - refphasor[0, 1], -122 - refphasor[1, 1], 118 - refphasor[2, 1]]
                    # if self.iteration > 59: #Change here if we want to set varying targets
                    #     Vtargdict[key]['Vmag'] = [0.92 - refphasor[0, 0], 0.92 - refphasor[0, 0],
                    #                               0.92 - refphasor[0, 0]]
                    #     Vtargdict[key]['Vang'] = [-4 - refphasor[0, 1], -4 - refphasor[0, 1], -4 - refphasor[0, 1]]
                        # if '671_a' and '671_b' and '671_c' not in Qsat_nodes:
                        #     Vtargdict[key]['Vmag'] = [0.96 - refphasor[0, 0], 0.96 - refphasor[0, 0],0.96 - refphasor[0, 0]]
                        # if '671_a' and '671_b' and '671_c' not in Psat_nodes:
                        #     Vtargdict[key]['Vang'] = [-2 - refphasor[0, 1], -2 - refphasor[0, 1], -2 - refphasor[0, 1]]

                    '''ICDI short-term fix
                    if '671_a' and '671_b' and '671_c' in Psat_nodes:
                        self.P_flag = []
                        self.P_flag.append(1)
                    if '671_a' and '671_b' and '671_c' in Qsat_nodes:
                        self.Q_flag = []
                        self.Q_flag.append(1)
                    '''
                    # elif self.iteration >= 26*2: #Change here if we want to set varying targets
                    #     if '671_a' and '671_b' and '671_c' not in Psat_nodes:
                    #         Vtargdict[key]['Vang'] = [-3 - refphasor[0, 1], -3 - refphasor[0, 1], -3 - refphasor[0, 1]]
                    #
                    #     if '671_a' and '671_b' and '671_c' not in Qsat_nodes:
                    #         Vtargdict[key]['Vmag'] = [0.94 - refphasor[0, 0], 0.94 - refphasor[0, 0],
                    #                                   0.94 - refphasor[0, 0]]
                    #     if '671_a' and '671_b' and '671_c' in Psat_nodes:
                    #         self.P_flag = []
                    #         self.P_flag.append(2)
                    #     if '671_a' and '671_b' and '671_c' in Qsat_nodes:
                    #         self.Q_flag = []
                    #         self.Q_flag.append(2)

                    # elif self.iteration >= 39: #Change here if we want to set varying targets
                    #      Vtargdict[key]['Vmag'] = [0.94 - refphasor[0, 0], 0.94 - refphasor[0, 0],0.94 - refphasor[0, 0]]
                    #      Vtargdict[key]['Vang'] = [-4 - refphasor[0, 1], -4 - refphasor[0, 1], -4 - refphasor[0, 1]]

                    '''ICDI short-term fix
                    if len(self.P_flag) > 0:
                        if self.P_flag[0] == 1:
                            Vtargdict[key]['Vang'] = [-2.5 - refphasor[0, 1], -2.5 - refphasor[0, 1], -2.5 - refphasor[0, 1]]
                        elif self.P_flag[0] == 2:
                            Vtargdict[key]['Vang'] = [-2 - refphasor[0, 1], -2 - refphasor[0, 1], -2 - refphasor[0, 1]]

                    if len(self.Q_flag) > 0:
                        if self.Q_flag[0] == 1:
                            Vtargdict[key]['Vmag'] = [0.95 - refphasor[0, 0], 0.95 - refphasor[0, 0],0.95 - refphasor[0, 0]]
                        if self.Q_flag[0] == 2:
                            Vtargdict[key]['Vmag'] = [0.95 - refphasor[0, 0], 0.95 - refphasor[0, 0],0.95 - refphasor[0, 0]]
                    '''
                    Vtargdict[key]['KVbase'] = [cons_kVbase[0],cons_kVbase[1],cons_kVbase[2]]
                    Vtargdict[key]['KVAbase'] = [cons_kVAbase[0],cons_kVAbase[1],cons_kVAbase[2]] #assumes 3ph sub

            computed_targets = {}

            #for key in act_keys: #DEBUG BELOW
            #for key, ibus in myfeeder.busdict.items():
            for key in lpbc_nodeIDs:
                if key in lpbc_nodes:
                    #lpbcID = 'lpbc_' + key
                    lpbcID = key
                    #intialize
                    computed_targets[lpbcID] = {}
                    #Vmag_prev = {}
                    #Vmag_prev[key] = np.ones((3,feeder_init.timesteps))*np.inf
                    computed_targets[lpbcID]['phase'] = []
                    computed_targets[lpbcID]['delV'] = []
                    computed_targets[lpbcID]['delta'] = []
                    computed_targets[lpbcID]['kvbase'] = []
                    computed_targets[lpbcID]['kvabase'] = []
                    
                    # TODO: shiff from line to phase channel tags
                    #for ph in ibus.phases:
                    for ph in lpbc_phases:
                        if ph == 'a':
                            phidx  = 0
                            computed_targets[lpbcID]['phase'].append('ph_A')
                            #computed_targets[lpbcID]['channels'].append('L1')
                            computed_targets[lpbcID]['delV'].append(Vtargdict[key]['Vmag'][phidx])
                            if angle_unit == 'radians':
                                computed_targets[lpbcID]['delta'].append(np.radians(Vtargdict[key]['Vang'][phidx]))
                            if angle_unit == 'degrees':
                                computed_targets[lpbcID]['delta'].append(Vtargdict[key]['Vang'][phidx])
                            computed_targets[lpbcID]['kvbase'].append(Vtargdict[key]['KVbase'][phidx])
                            computed_targets[lpbcID]['kvabase'].append(Vtargdict[key]['KVAbase'][phidx])
                            
                            #Vmag_prev[key] = np.ones((3,feeder_init.timesteps))*np.inf

                        if ph == 'b':
                            phidx  = 1
                            computed_targets[lpbcID]['phase'].append('ph_B')
                            #computed_targets[lpbcID]['channels'].append('L2')
                            computed_targets[lpbcID]['delV'].append(Vtargdict[key]['Vmag'][phidx])
                            if angle_unit == 'radians':
                                computed_targets[lpbcID]['delta'].append(np.radians(Vtargdict[key]['Vang'][phidx]))
                            if angle_unit == 'degrees':
                                computed_targets[lpbcID]['delta'].append(Vtargdict[key]['Vang'][phidx])
                            computed_targets[lpbcID]['kvbase'].append(Vtargdict[key]['KVbase'][phidx])
                            computed_targets[lpbcID]['kvabase'].append(Vtargdict[key]['KVAbase'][phidx])

                        if ph == 'c':
                            phidx  = 2
                            computed_targets[lpbcID]['phase'].append('ph_C')
                            #computed_targets[lpbcID]['channels'].append('L3')
                            computed_targets[lpbcID]['delV'].append(Vtargdict[key]['Vmag'][phidx])
                            if angle_unit == 'radians':
                                computed_targets[lpbcID]['delta'].append(np.radians(Vtargdict[key]['Vang'][phidx]))
                            if angle_unit == 'degrees':
                                computed_targets[lpbcID]['delta'].append(Vtargdict[key]['Vang'][phidx])
                            computed_targets[lpbcID]['kvbase'].append(Vtargdict[key]['KVbase'][phidx])
                            computed_targets[lpbcID]['kvabase'].append(Vtargdict[key]['KVAbase'][phidx])



                
            # loop through the computed targets and send them to all LPBCs:
            for lpbc_name, targets in computed_targets.items():
                print(f'announcing to lpbc: {lpbc_name}')
                await self.broadcast_target(lpbc_name, targets['phase'], \
                                targets['delV'], targets['delta'], targets['kvbase'], kvabases=targets['kvabase']) #kvabases=targets['kvabase']

if len(sys.argv) > 1:
    cfg = config_from_file(sys.argv[1])
else:
    sys.exit("Must supply config file as argument: python3 spbc.py <config file.toml>")
spbc_instance = myspbc(cfg)
run_loop()
