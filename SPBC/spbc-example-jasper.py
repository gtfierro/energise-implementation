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

#init feeder & other vars
phase_size, feeder_init = feeder_init()
print('phases on network:',phase_size)

# SETTINGS
lpbc_phases = ['a','b','c']

dummy_ref = True
constant_phasor = True

if dummy_ref == True:
    print('WARNING: dummy_ref ON')
if constant_phasor == True:
    # set phasor target values here (not relative)
    cons_Vmag = [0.9862920,0.9956446,0.9881567]
    cons_Vang = [-1.61526,-121.75103,118.20174]
    cons_kVbase = np.ones(3)*4.16/np.sqrt(3)
    cons_kVAbase = np.ones(3)*5000/3
    print('WARNING: constant_phasor ON')

# TODO: vmagprev, check dims across all instances, think it shoud just be 3
Vmag_prev = []
#Vmag_prev = {}
#Vmag_prev[key] = np.ones((3))*np.inf

# TODO; put lpbc_nodes outsie of loop in init section?
#lpbc_nodes = []

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
        schedule(self.call_periodic(60, self.compute_and_announce))
        ### set some refphasor variable == true/false to determine length of schedule

    async def compute_and_announce(self):
        print('')
        print('~~~ New compute_and_announce instance ~~~')
        
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
                
                # get perf nodes (lpbc nodes)
                for key, ibus in feeder_init.busdict.items():
                    if lpbc == 'lpbc_' + key:
                        lpbc_nodes.append(key)
        
        # create list of nodes where ICDI is true (Change to distinguish b/w P & Q)
                if status['pSaturated'] == True:
                    Psat_nodes.append(lpbc[5:])
                if status['qSaturated'] == True:
                    Qsat_nodes.append(lpbc[5:])
                    
        # hardcode lpbc_nodes in
        lpbc_nodes = ['671']
        
        print('Psat',Psat_nodes)
        print('Qsat',Qsat_nodes)
        
        # ~~ REFERENCE PHASOR ~~ #
        # num of ref channels must match num of phases for slack bus on impedance model
         # otherwise read ref phasor error is returned
        refphasor_init = np.ones((phase_size,2))*np.inf
        refphasor = refphasor_init
        # how to loop through all reference phasor channels
        
        for channel, data in self.reference_phasors.items():
            print(f"Channel {channel} has {len(data) if data else 0} points")
            if data != None:
                #store most recent uPMU ref phasor values
                if 'L1' in channel:
                    refphasor[0,0] = data[-1]['magnitude']
                    refphasor[0,1] = data[-1]['angle']
                
                #if 'L2' in channel:
                #    refphasor[1,0] = data[-1]['magnitude']
                #    refphasor[1,1] = data[-1]['angle']
                #if 'L3' in channel:
                #    refphasor[2,0] = data[-1]['magnitude']
                #    refphasor[2,1] = data[-1]['angle']
                
                # TODO: phase allocation?
                    
                if 'L2' in channel:
                    refphasor[1,0] = data[-1]['magnitude']
                    refphasor[1,1] = data[-1]['angle']
                if 'L3' in channel:
                    refphasor[2,0] = data[-1]['magnitude']
                    refphasor[2,1] = data[-1]['angle']
        
            
        #convert Vmag to p.u. (subKVbase_phg defined in main)
        refphasor[:,0] = refphasor[:,0]/(subkVbase_phg*1000) # TODO: compute refphasor vmag correctly
        #convert angle from degrees to rads
        # TODO: phases in right order?
        #[[1.00925961 2.04308987]
        #[1.00899569 6.2332654 ]
        #[1.01064548 4.13935041]]
        refphasor[:,1] = refphasor[:,1]*np.pi/180 # TODO: change phB to -120 first?
        print('phasor reference [pu-rad]:')
        print(refphasor)

        if dummy_ref == True:
            #dummy values
            refphasor = refphasor_init
            refphasor[:,0]=1
            refphasor[:,1]=[0,4*np.pi/3,2*np.pi/3]
            print('using dummy reference values')
                         
        
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
            Vtargdict, act_keys, subkVAbase, myfeeder = spbc_run(refphasor,Psat_nodes,Qsat_nodes,lpbc_nodes)   
            
            # TODO: how do we communicate phase information?
            # None-padded? dicts keyed by the channel name?
            # should set computed targets to have lpbc_nodeID so they dont have to be ordered specifically
    
            if constant_phasor == True:
                Vtargdict = {}
                refphasor[:,1] = refphasor[:,1]*180/np.pi
                refphasor[1,1] = refphasor[1,1]-360
                for key in lpbc_nodes:
                    Vtargdict[key] = {}
                    Vtargdict[key]['Vmag'] = [cons_Vmag[0]-refphasor[0,0],cons_Vmag[1]-refphasor[1,0],cons_Vmag[0]-refphasor[2,0]]
                    Vtargdict[key]['Vang'] = [cons_Vang[0]-refphasor[0,1],cons_Vang[1]-refphasor[1,1],cons_Vang[2]-refphasor[2,1]]
                    Vtargdict[key]['KVbase'] = [cons_kVbase[0],cons_kVbase[1],cons_kVbase[2]]
                    Vtargdict[key]['KVAbase'] = [cons_kVAbase[0],cons_kVAbase[1],cons_kVAbase[2]] #assumes 3ph sub
                    
            computed_targets = {}
            
            #for key in act_keys:
            for key, ibus in myfeeder.busdict.items():
                if key in lpbc_nodes:
                    lpbcID = 'lpbc_' + key
                    #intialize
                    computed_targets[lpbcID] = {}
                    #Vmag_prev = {}
                    #Vmag_prev[key] = np.ones((3,feeder_init.timesteps))*np.inf
                    computed_targets[lpbcID]['channels'] = []
                    computed_targets[lpbcID]['V'] = []
                    computed_targets[lpbcID]['delta'] = []
                    computed_targets[lpbcID]['kvbase'] = []
                    computed_targets[lpbcID]['kvabase'] = []
                    
                    #for ph in ibus.phases:
                    for ph in lpbc_phases:
                        if ph == 'a':
                            phidx  = 0
                            computed_targets[lpbcID]['channels'].append('L1')
                            computed_targets[lpbcID]['V'].append(Vtargdict[key]['Vmag'][phidx])
                            computed_targets[lpbcID]['delta'].append(Vtargdict[key]['Vang'][phidx])
                            computed_targets[lpbcID]['kvbase'].append(Vtargdict[key]['KVbase'][phidx])
                            computed_targets[lpbcID]['kvabase'].append(Vtargdict[key]['KVAbase'][phidx])
                            
                            #Vmag_prev[key] = np.ones((3,feeder_init.timesteps))*np.inf
                        if ph == 'b':
                            phidx  = 1
                            computed_targets[lpbcID]['channels'].append('L2')
                            computed_targets[lpbcID]['V'].append(Vtargdict[key]['Vmag'][phidx])
                            computed_targets[lpbcID]['delta'].append(Vtargdict[key]['Vang'][phidx])
                            computed_targets[lpbcID]['kvbase'].append(Vtargdict[key]['KVbase'][phidx])
                            computed_targets[lpbcID]['kvabase'].append(Vtargdict[key]['KVAbase'][phidx])
                        
                        if ph == 'c':
                            phidx  = 2
                            computed_targets[lpbcID]['channels'].append('L3')
                            computed_targets[lpbcID]['V'].append(Vtargdict[key]['Vmag'][phidx])
                            computed_targets[lpbcID]['delta'].append(Vtargdict[key]['Vang'][phidx])
                            computed_targets[lpbcID]['kvbase'].append(Vtargdict[key]['KVbase'][phidx])
                            computed_targets[lpbcID]['kvabase'].append(Vtargdict[key]['KVAbase'][phidx])
                            
                        
                    
                
            # loop through the computed targets and send them to all LPBCs:
            for lpbc_name, targets in computed_targets.items():
                await self.broadcast_target(lpbc_name, targets['channels'], \
                                targets['V'], targets['delta'], targets['kvbase'], kvabases=targets['kvabase']) #kvabases=targets['kvabase']

if len(sys.argv) > 1:
    cfg = config_from_file(sys.argv[1])
else:
    sys.exit("Must supply config file as argument: python3 spbc.py <config file.toml>")
spbc_instance = myspbc(cfg)
run_loop()
