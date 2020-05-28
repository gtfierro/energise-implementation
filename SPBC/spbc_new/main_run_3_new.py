# THIS IS THE INTERFACE FOR THE S-PBC
#SPBC-HIL files moved 7/15/19

# In[1]:
    
from setup_3 import *
from constraints_3 import *
from dss_3 import *
import datetime
import time

# In[2]:
ts = time.time()
print()
print('SPBC running...',ts)
print(datetime.datetime.fromtimestamp(ts))

# In[3]: READ FILE PATHS

# Enter the path/name of the model's excel file and import
# Enter the path/name of the load file and import

'IEEE13_UNBALANCED'
# =============================================================================
# filepath = "IEEE13/"
# modelpath = filepath + "001 phasor08_IEEE13_OPAL.xls"
# loadfolder = "IEEE13/"
# loadpath = loadfolder + "001_phasor08_IEEE13_T12-3.xlsx"
# =============================================================================

'IEEE13_BALANCED'
# filepath = "IEEE13_bal/"
# modelpath = filepath + "016_GB_IEEE13_balance_reform.xlsx"
# loadfolder = "IEEE13_bal/"
# loadpath = loadfolder + "016_GB_IEEE13_balance_norm03.xlsx"


'UCB33'
# =============================================================================
filepath = "UCB33/"
modelpath = filepath + "005_GB_UCB33_opal_v3.xlsx"
loadfolder = "UCB33/"
loadpath = loadfolder + "005_GB_UCB33_time_sigBuilder_Q_13_14_norm03.xlsx"
# =============================================================================


'PL0001_v2'
# =============================================================================
# filepath = "PL0001_v2/"
# modelpath = filepath + "PL0001_OPAL_working_reform_xfmr.xlsx"
# #modelpath = filepath + "PL0001_OPAL_working_reform_notrans_F.xlsx"
# loadfolder = "PL0001_v2/"
# loadpath = loadfolder + "PL0001_July_Q_F.xlsx"
# =============================================================================

# =============================================================================
# filepath = "IEEE37/"
# modelpath = filepath + "003_GB_IEEE37_OPAL_reform2.xls"
# 
# loadfolder = "IEEE37/TD/"
# loadpath = loadfolder + "003_GB_IEEE37_time_sigBuilder_Q_F_act1.xlsx"
# =============================================================================



#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
# In[4]: ENTER TEST SETTINGS

# Specify substation kV, kVA bases, and the number of timesteps in the load data
'IEEE13'
# subkVbase_phg = 4.16/np.sqrt(3)
# subkVAbase = 5000.
# timesteps = 3 #int((16-8)*60)

'UCB33'
# =============================================================================
subkVbase_phg = 12.47/np.sqrt(3)
subkVAbase = 3000.
timesteps = 2 #int((16-8)*60)
# =============================================================================

'PL0001'
# =============================================================================
# subkVbase_phg = 12.6/np.sqrt(3)
# subkVAbase = 1500. #1000000.  #1500000.
# timesteps = 2
# =============================================================================

plot = 0 #turn plot on/off

# Specify initial timestep
date = datetime.datetime.now()
month, day, hour, minute = date.month, date.day, date.hour, date.minute
#timestepcur = hour*60+minute
timestepcur = 0 #int(11*60) # [INPUT HERE] Manual input of start time

# Input atcuator nodes and capacities ('nodeID': phases, kVA capacity)
act_init = { '18': {'a': 500, 'b': 500, 'c':500}
        # '652': {'a': 200, 'b': 0, 'c':0},
        # '671': {'a': 100, 'b': 0, 'c':0},
        # '692': {'a': 200, 'b': 0, 'c':0}
            }

# Input constnats for PV forecasting
PV_on = False # True for ON
PVnodes = []

PVforecast = {}
#PVforecast['on_off'] = PV_on
for node in PVnodes: # this sets all nodes the same, would have to manually change fields to have different inputs for different nodes
    PVforecast[node] = {}
    PVforecast[node]['on_off'] = PV_on
    PVforecast[node]['lat'] = 37.87
    PVforecast[node]['lon'] = -122
    PVforecast[node]['maridian'] = -120
    PVforecast[node]['PVfrac'] = 0.3

# In[5]: CREATE FEEDER OBJECT

# to allow operations on the feeder without running the optimization
# basic feeder init such that model is read in and network is defined
def feeder_init(Psat_nodes=[],Qsat_nodes=[],timestepcur=timestepcur):
    modeldata = pd.ExcelFile(modelpath)
    actpath = loadpath
    
    # set dummy values for undefined variables
    date = datetime.datetime.now()
    month = date.month
    day = date.day
    hour = date.hour
    minute = date.minute
    #timestepcur = hour*60+minute
    timestepcur = timestepcur
    
    Psat_nodes = []
    Qsat_nodes = []
    
    refphasor = np.ones((3,2))
    refphasor[:,0]=1
    refphasor[:,1]=[0,4*np.pi/3,2*np.pi/3]
    
    #get feeder
    feeder_init = feeder(modelpath,loadfolder,loadpath,actpath,timesteps,timestepcur,
                         subkVbase_phg,subkVAbase,refphasor,Psat_nodes,Qsat_nodes,PVforecast,act_init)
    feeder_init
    
    phase_size = 0
    for key, inode in feeder_init.busdict.items():
        if inode.type == 'SLACK' or inode.type == 'Slack' or inode.type == 'slack':
            for ph in inode.phases:
                phase_size += 1
    
    return phase_size, feeder_init

# In[6]: DEFINTE SPBC RUN - OPTIMIZATION SETTINGS

    ## WORKSPACE: CURRENT MODEL FOR TARGET GENERATION ##
def spbc_run(refphasor,Psat_nodes,Qsat_nodes,perf_nodes,timestepcur): #write 'none' if doesnt exist    
    
    modeldata = pd.ExcelFile(modelpath) 
    actpath = loadpath
    
    # set tuning parameters of objectives: 0 = off
    # lam1 - phasor target, lam2 - phase balancing, lam3 - voltage volatility
    # lamcost - const function, lamdistpf - transmission/distribution power flow
    lam1 = 1    # phasor target matching
    lam2 = 0    # phase balancing
    lam3 = 0    # voltage volatility
    lam4 = 0    # nominal voltages
    lamInj = 0  # objective 5 (min injections)
    lamLineFlow = 0
    
    lamcost = 1
    lamdistpf = 50
    
    # turn additional objectives on/off
    costfn_on_off = 0 # 0=off, 1=on  
    distpf_on_off = 0 # 0=off, 1=on
    
# phasor target settings:
    if lam1 > 0:
        target_key = '671'
        Vmag_match = [.99]*3
        Vang_match = [0 - np.radians(1), 4/3*np.pi - np.radians(1), 2/3*np.pi - np.radians(1)] 
    
# phase balancing settings:
    # no settings to change for this objective
    
# voltage volatility settings:
    if lam3 > 0:
    # choose locations where you wish to control volatility:
        # either 'all', 'none', or a list of target nodes (i.e. ['680','692'])
        target_loc = []
    
    # cost function settings:
    if costfn_on_off == 1:
        actcostdata = pd.read_excel(actcostpath, index_col=0)
        
    # power flow target settings:
    if distpf_on_off == 1:      
        '# ~~~ INPUT NODE ~~~ #'  
        nodeloc = 77 # [INPUT HERE]
        '______________________'
        
        dfdtarg = pd.read_csv(genpath, index_col = 0)
        dfdtarg.index += 1
        Pstr, Qstr, Pkey, Qkey = f'{nodeloc}_P', f'{nodeloc}_Q', [], []
        for head in dfdtarg.columns:
            if Pstr in head:
                Pkey.append(head)
            if Qstr in head:
                Qkey.append(head)
        netpflow_targ = np.transpose(dfdtarg[Pkey].values/subkVAbase)
        netqflow_targ = np.transpose(dfdtarg[Qkey].values/subkVAbase)
        print(netpflow_targ[:][:][0:3]*subkVAbase)
        print(netqflow_targ[:][:][0:3]*subkVAbase)
        
    else:
        lamdistpf = 0
    
    # Create feeder object
    myfeeder = feeder(modelpath,loadfolder,loadpath,actpath,timesteps,timestepcur,
                         subkVbase_phg,subkVAbase,refphasor,Psat_nodes,Qsat_nodes,PVforecast,act_init)    
    myfeeder
    
    # In[7]: DEFINTE SPBC RUN - OBJECTIVES
    
    # Run optimization problem and generate targets
    
    
    obj = 0
    #pdb.set_trace()
    Vangnom = [0, 4/3*np.pi, 2/3*np.pi] 
    
    for ts in range(0,myfeeder.timesteps):
        for key, bus in myfeeder.busdict.items():
        #for key, bus in myfeeder.busdict['633']:
            #print(bus.name)
            
    # objective 1 - phasor target  
            if lam1 > 0:
                if key == target_key:
                    '3phase'
# =============================================================================
#                     if (bus.phasevec == np.ones((3,timesteps))).all():
#                         obj = obj + lam1*((cp.square(bus.Vang_linopt[0,ts]-Vang_match[0]) + cp.square(bus.Vang_linopt[1,ts]-Vang_match[1]) + cp.square(bus.Vang_linopt[2,ts]-Vang_match[2])))
#                         obj += lam1*((cp.square(bus.Vmagsq_linopt[0,ts]-Vmag_match[0]**2) + cp.square(bus.Vmagsq_linopt[1,ts]-Vmag_match[1]**2) + cp.square(bus.Vmagsq_linopt[2,ts]-Vmag_match[2]**2)))
# =============================================================================
                    'T12.4'
                    obj += lam1*(cp.square(bus.Vang_linopt[0,ts]-Vang_match[0]))
                    obj += lam1*(cp.square(bus.Vmagsq_linopt[0,ts]-Vmag_match[0]**2))

    # objective 2 - phase balancing
            if lam2 > 0:
                if (bus.phasevec == np.array([[1],[1],[0]])).all():
                    obj = obj + lam2*cp.square(bus.Vmagsq_linopt[0,ts]-bus.Vmagsq_linopt[1,ts])
                    obj = obj + lam2*cp.square((bus.Vang_linopt[0,ts]-bus.Vang_linopt[1,ts])-(Vangnom[0]-Vangnom[1]))
                if (bus.phasevec == np.array([[1],[0],[1]])).all():
                    obj = obj + lam2*cp.square(bus.Vmagsq_linopt[0,ts]-bus.Vmagsq_linopt[2,ts])
                    obj = obj + lam2*cp.square((bus.Vang_linopt[0,ts]-bus.Vang_linopt[2,ts])-(Vangnom[0]-Vangnom[2]))
                if (bus.phasevec == np.array([[0],[1],[1]])).all():
                    obj = obj + lam2*cp.square(bus.Vmagsq_linopt[1,ts]-bus.Vmagsq_linopt[2,ts])
                    obj = obj + lam2*cp.square((bus.Vang_linopt[1,ts]-bus.Vang_linopt[2,ts])-(Vangnom[1]-Vangnom[2]))
                if (bus.phasevec == np.ones((3,timesteps))).all():
                    obj = obj + lam2*(cp.square(bus.Vmagsq_linopt[0,ts]-bus.Vmagsq_linopt[1,ts]) +
                                      cp.square(bus.Vmagsq_linopt[0,ts]-bus.Vmagsq_linopt[2,ts]) +
                                      cp.square(bus.Vmagsq_linopt[1,ts]-bus.Vmagsq_linopt[2,ts]))
                    obj = obj + lam2*(cp.square((bus.Vang_linopt[0,ts]-bus.Vang_linopt[1,ts])-(Vangnom[0]-Vangnom[1])) +
                                      cp.square((bus.Vang_linopt[0,ts]-bus.Vang_linopt[2,ts])-(Vangnom[0]-Vangnom[2])) +
                                      cp.square((bus.Vang_linopt[1,ts]-bus.Vang_linopt[2,ts])-(Vangnom[1]-Vangnom[2])))
                    
    # objective 4 - target all nominal - currently set up only for 3phase networks
            if lam4 > 0:
# =============================================================================
#                 if (bus.phasevec == np.array([[1],[1],[0]])).all():
#                     obj = obj + lam2*(cp.square(1-bus.Vmagsq_linopt[0,ts]) + 
#                                       cp.square(1-bus.Vmagsq_linopt[1,ts]))
#                     obj = obj + lam2*cp.square((bus.Vang_linopt[0,ts]-bus.Vang_linopt[1,ts])-(Vangnom[0]-Vangnom[1]))
#                 if (bus.phasevec == np.array([[1],[0],[1]])).all():
#                     obj = obj + lam2*(cp.square(1-bus.Vmagsq_linopt[0,ts]) + 
#                                       cp.square(1-bus.Vmagsq_linopt[2,ts]))
#                     obj = obj + lam2*cp.square((bus.Vang_linopt[0,ts]-bus.Vang_linopt[2,ts])-(Vangnom[0]-Vangnom[2]))
#                 if (bus.phasevec == np.array([[0],[1],[1]])).all():
#                     obj = obj + lam2*(cp.square(1-bus.Vmagsq_linopt[1,ts]) + 
#                                       cp.square(1-bus.Vmagsq_linopt[2,ts]))
#                     obj = obj + lam2*cp.square((bus.Vang_linopt[1,ts]-bus.Vang_linopt[2,ts])-(Vangnom[1]-Vangnom[2]))
# =============================================================================
                if (bus.phasevec == np.ones((3,timesteps))).all():
                    obj += lam4*(cp.square(1-bus.Vmagsq_linopt[0,ts]) + 
                                 cp.square(1-bus.Vmagsq_linopt[1,ts]) +
                                 cp.square(1-bus.Vmagsq_linopt[2,ts]))
                    obj += lam4*(cp.square(Vangnom[0]-bus.Vang_linopt[0,ts]) +
                                 cp.square(Vangnom[1]-bus.Vang_linopt[1,ts]) +
                                 cp.square(Vangnom[2]-bus.Vang_linopt[2,ts]))

    # objective 3 - voltage volitility
    if lam3 > 0:
        for ts in range(1,myfeeder.timesteps):
            for key, bus in myfeeder.busdict.items():
                if isinstance(target_loc,list):
                    for targkey in target_loc:
                        if bus.name == 'bus'+targkey:
                            obj += lam3*cp.square(bus.Vmagsq_linopt[0,ts]-bus.Vmagsq_linopt[0,ts-1]+
                                                  bus.Vmagsq_linopt[1,ts]-bus.Vmagsq_linopt[1,ts-1]+
                                                  bus.Vmagsq_linopt[2,ts]-bus.Vmagsq_linopt[2,ts-1])
                else:
                    obj += lam3*cp.square(bus.Vmagsq_linopt[0,ts]-bus.Vmagsq_linopt[0,ts-1]+
                                          bus.Vmagsq_linopt[1,ts]-bus.Vmagsq_linopt[1,ts-1]+
                                          bus.Vmagsq_linopt[2,ts]-bus.Vmagsq_linopt[2,ts-1])
                            
    # voltage volatility doesn't really make sense for only 2 timesteps? compute over horizon? 
    # OR (preferred) find way to store value from previous iteration?
        # is this equivalent even though not all cvx vars anymore?
        # initialize by minimizing volatility over first 10 timesteps...
    # alternative method:
        # consider previous timestep as well as next X timesteps
        # weigh previous timestep more heavily than future timesteps
    
    # TODO: vmagprev
    #Vmag_prev = {}
    #Vmag_prev[key] = np.ones((3,myfeeder.timesteps))
    # objective 3.2 [HIL} - voltage volatility
    '''
    for ts in range(0,myfeeder.timesteps):
        for key, bus in myfeeder.busdict.items():
            if key in perf_nodes:
                if bus.name == 'bus_' + key:
                    obj += lam3*cp.square(bus.Vmagsq_linopt[0,ts]-Vmag_prev[key][0]+
                                          bus.Vmagsq_linopt[1,ts]-Vmag_prev[key][1]+
                                          bus.Vmagsq_linopt[2,ts]-Vmag_prev[key][2])
    '''
    #objective 5 - minimize power injections
    if lamInj > 0:
        for ts in range(0,myfeeder.timesteps):
            for idx in range(3):
                for key, iact in myfeeder.actdict.items():
                    # [TODO] subtract loads from injections
                    try:
                        iload = iact.node.loads[0]
                        Pld = iload.Psched[idx,ts]/iact.node.kVAbase
                        Qld = iload.Qsched[idx,ts]/iact.node.kVAbase
                        #print(f'found load at {key} - P {Pld}, Q {Qld}')
                    except:
                        Pld, Qld = 0, 0
                        #print(f'no load at {key} - P {Pld}, Q {Qld}')
                    # add Pgen and Qgen values
                    obj += lamInj * (cp.square(iact.Pgen[idx,ts]-Pld)+cp.square(iact.Qgen[idx,ts]-Qld))
                for key, ishunt in myfeeder.shuntdict.items():
                    # add Pgen and Qgen values
                    if ishunt.id[0:8] == 'sync_cap':
                        try:
                            iload = ishunt.node.loads[0]
                            Qld = iload.Qsched[idx,ts]/iact.node.kVAbase
                            #print(f'found load at {key} - Q {Qld} (shunt)')
                        except:
                            Pld, Qld = 0, 0
                            #print(f'no load at {key} - Q {Qld} (shunt)')
                        obj += lamInj * (cp.square(ishunt.Qgen[idx,ts]-Qld))
                slack_check = []
                for key, iline in myfeeder.linedict.items():   
                    if iline.from_node.type == 'SLACK':
                        # assumes there is no load at the slack
                        obj += lamInj * (cp.square(iline.P_linopt[idx,ts])+cp.square(iline.Q_linopt[idx,ts]))
                        slack_check.append(1)
                if not slack_check:
                    print('NO SLACK BUS FOUND')

    if lamLineFlow > 0:
        for ts in range(0,myfeeder.timesteps):       
            for idx in range(3):
                for key, iline in myfeeder.linedict.items():
                    capacity = iline.MVArating_3ph/3*1000/iline.from_node.kVAbase
                    obj += lamLineFlow * (cp.square(iline.P_linopt[idx,ts])+cp.square(iline.Q_linopt[idx,ts]))/(iline.MVArating_3ph/3*1000/iline.from_node.kVAbase)
                    #obj += lamLineFlow * (cp.square(iline.P_linopt[idx,ts])+cp.square(iline.Q_linopt[idx,ts]))
    
                
    # add cost function to actuators       
    if costfn_on_off == 1:
        for ts in range(0,myfeeder.timesteps):  
                    
            for key, inode in myfeeder.busdict.items():
                for iact in inode.actuators:
                    key_str = str(key)
                    for idx in range(0,3):
                        obj += cp.square(iact.Pgen[idx,ts:ts+1] * actcostdata[key_str][ts])
                        
    # distribution level power flow objective to meet transmission SPBC target
        # in this iteration, it is done as meeting power flows directly defined by the transmission level SPBC
        # but in later iterations the goal is to have an intermediary feedback controller that translates the voltage target
        # into a power flow
        
    if distpf_on_off == 1:
                
        '# ~~~ SLACK LINE FLOW ~~~ #'
        for key, inode in myfeeder.busdict.items():
            if inode.type == 'SLACK' or inode.type == 'Slack' or inode.type == 'slack':
                conn = inode.edges_out
                break
        for ts in range(0,myfeeder.timesteps):
             for idx in range(0,3):
                 obj += lamdistpf*cp.square(conn[0].P_linopt[idx,ts:ts+1] - netpflow_targ[idx,ts:ts+1])
                 obj += lamdistpf*cp.square(conn[0].Q_linopt[idx,ts:ts+1] - netqflow_targ[idx,ts:ts+1])


    '~~~~~~~~~~~~~~~~'
    'CVX OPT SETTINGS'
    '~~~~~~~~~~~~~~~~'
     
    objective = cp.Minimize(obj)
    constraints = cvx_set_constraints(myfeeder,1) # Second argument turns actuators on/off
    prob = cp.Problem(objective, constraints)
    #result = prob.solve()
    result = prob.solve(verbose=False,eps_rel=1e-5,eps_abs=1e-10, max_iter = 50000)
    #result = prob.solve(verbose=False,eps_rel=1e-10,eps_abs=1e-10, max_iter = 50000)
    print(result)
    
    # In[7]:
    
    DSS_alltimesteps(myfeeder,1) # Second argument turns voltage alarms on/off
    
    'turn export targets on/off here'
    export_Vtargets(myfeeder)
    ##[jasper] - fn to get target in vector format for LPBC
    #nodes_arr,Vmag_targ,Vang_targ,KVbase = get_targets(myfeeder)
    
    # Vtargdict[key(nodeID)][Vmag/Vang/KVbase]
    Vtargdict, act_keys = get_targets(myfeeder)
    #lpbc_keys = [lpbc_node1,lpbc_node2,lpbc_node3,lpbc_node4]
    return Vtargdict, act_keys, subkVAbase, myfeeder


# In[8]:
# Run main_run - TURN OFF FOR REAL TIME SOLVING
    

### dummy values ###    
Psat = []
Qsat = []
perf_nodes = []
#create dummy refphasor of nominal voltages
refphasor = np.ones((3,2))
refphasor[:,0]=1
refphasor[:,1]=[0,4/3*np.pi,2/3*np.pi]
#refphasor[:,1]=[0,2*np.pi/3,4*np.pi/3]

Vtargdict, act_keys, subkVAbase, myfeeder = spbc_run(refphasor,Psat,Qsat,perf_nodes,timestepcur)
gendict, shuntgendict = get_gen(myfeeder), get_shunt(myfeeder) # (+) is injecting, (-) is consuming
flowdict = get_flow(myfeeder)
'''
#tf = time.time()
#print('time to load model')
#print(tf-ts)
'''

# In[9]:
# Plot first timestep of result

tsp = 0 # select timestep for convergence plot

# DSS_alltimesteps(myfeeder,1) 
plot = 1 #turn plot on/off
def plot_results():
    if plot == 1:
        import matplotlib.pyplot as plt
        # Plot lin result
        print('Linear sln')
        ph1 = []
        ph2 = []
        ph3 = []
        for key, bus in myfeeder.busdict.items():
            if bus.Vmagsq_linopt[0,0].value > 0:
                ph1.append(np.sqrt(bus.Vmagsq_linopt[0,0].value))
            else:
                ph1.append(0)
            if bus.Vmagsq_linopt[1,0].value > 0:
                ph2.append(np.sqrt(bus.Vmagsq_linopt[1,0].value))
            else:
                ph2.append(0)
            if bus.Vmagsq_linopt[2,0].value > 0:
                ph3.append(np.sqrt(bus.Vmagsq_linopt[2,0].value))
            else:
                ph3.append(0)
        
        
        plt.plot(ph1,'ro', label='ph1')
        plt.plot(ph2,'go', label='ph2')
        plt.plot(ph3,'bo', label='ph3')
        plt.ylabel('Vmag [p.u.]')
        plt.xlabel('Node')
        plt.ylim((0.8, 1.1))
        plt.legend()
        plt.show()
        
        print('Nonlinear sln')
        
        # Plot NL actuation result
        ph1 = list()
        ph2 = list()
        ph3 = list()
        for key, bus in myfeeder.busdict.items():
            ph1.append(bus.Vmag_NL[0,0]/(bus.kVbase_phg*1000))
            ph2.append(bus.Vmag_NL[1,0]/(bus.kVbase_phg*1000))
            ph3.append(bus.Vmag_NL[2,0]/(bus.kVbase_phg*1000))
        
        plt.plot(ph1,'ro', label='ph1')
        plt.plot(ph2,'go', label='ph2')
        plt.plot(ph3,'bo', label='ph3')
        plt.ylabel('Vmag [p.u.]')
        plt.xlabel('Node')
        plt.ylim((0.8, 1.1))
        #plt.ylim((0, 2))
        plt.legend()
        plt.show()
        
        print('Vang linear')
        
        ph1 = list()
        ph2 = list()
        ph3 = list()
        for key, bus in myfeeder.busdict.items():
            if bus.Vang_linopt[0,0].value == 0:
                ph1.append(-10)
            else:
                ph1.append(bus.Vang_linopt[0,0].value)
            if bus.Vang_linopt[1,0].value == 0:
                ph2.append(-10)
            else:
                ph2.append(bus.Vang_linopt[1,0].value)
            if bus.Vang_linopt[2,0].value == 0:
                ph3.append(-10)
            else:
                ph3.append(bus.Vang_linopt[2,0].value)
        
        plt.plot(ph1,'ro', label='ph1')
        plt.plot(ph2,'go', label='ph2')
        plt.plot(ph3,'bo', label='ph3')
        plt.ylabel('Vang [rad]')
        plt.xlabel('Node')
        plt.ylim((-1, 6))
        plt.legend()
        plt.show()
        
        print('Vang nonlinear')
        
        ph1 = list()
        ph2 = list()
        ph3 = list()
        for key, bus in myfeeder.busdict.items():
            if bus.Vang_NL[0,0] == 0:
                ph1.append(-10)
            else:
                ph1.append(bus.Vang_NL[0,0]*np.pi/180)
            if bus.Vang_NL[1,0] == 0:
                ph2.append(-10)
            else:
                ph2.append((bus.Vang_NL[1,0]*-2)*np.pi/180)
            if bus.Vang_NL[2,0] == 0:
                ph3.append(-10)
            else:
                ph3.append(bus.Vang_NL[2,0]*np.pi/180)
        
        plt.plot(ph1,'ro', label='ph1')
        plt.plot(ph2,'go', label='ph2')
        plt.plot(ph3,'bo', label='ph3')
        plt.ylabel('Vang [rad]')
        plt.xlabel('Node')
        plt.ylim((-1, 6))
        plt.legend()
        plt.show()
        
        #plot difference
        
        print('Vmag convergence')
        ph1 = list()
        ph2 = list()
        ph3 = list()
        for key, bus in myfeeder.busdict.items():
            if np.abs(bus.Vmag_NL[0,tsp]) and bus.kVbase_phg > 0:
                ph1.append(100*np.abs((np.sqrt(bus.Vmagsq_linopt[0,tsp].value)-bus.Vmag_NL[0,tsp]/(bus.kVbase_phg*1000))/(bus.Vmag_NL[0,tsp]/(bus.kVbase_phg*1000))))
            if np.abs(bus.Vmag_NL[1,tsp]) > 0:
                ph2.append(100*np.abs((np.sqrt(bus.Vmagsq_linopt[1,tsp].value)-bus.Vmag_NL[1,tsp]/(bus.kVbase_phg*1000))/(bus.Vmag_NL[1,tsp]/(bus.kVbase_phg*1000))))
            if np.abs(bus.Vmag_NL[2,tsp]) > 0:
                ph3.append(100*np.abs((np.sqrt(bus.Vmagsq_linopt[2,tsp].value)-bus.Vmag_NL[2,tsp]/(bus.kVbase_phg*1000))/(bus.Vmag_NL[2,tsp]/(bus.kVbase_phg*1000))))
        
        plt.plot(ph1,'ro', label='ph1')
        plt.plot(ph2,'go', label='ph2')
        plt.plot(ph3,'bo', label='ph3')
        plt.ylabel('Vmag difference [%]')
        plt.xlabel('Node')
        plt.ylim((-.1, 10))
        plt.legend()
        plt.show()
        
        print('Vang convergence')
        
        ph1 = list()
        ph2 = list()
        ph3 = list()
        for key, bus in myfeeder.busdict.items():
            # % dif
            #ph1.append(np.abs((bus.Vang_linopt[0,0].value-bus.Vang_NL[0,0]*np.pi/180)/(bus.Vang_NL[0,0]*np.pi/180)))
            #ph2.append(np.abs((bus.Vang_linopt[1,0].value-(bus.Vang_NL[1,0]*-2)*np.pi/180)/((bus.Vang_NL[1,0]*-2)*np.pi/180)))
            #ph3.append(np.abs((bus.Vang_linopt[2,0].value-bus.Vang_NL[2,0]*np.pi/180)/(bus.Vang_NL[2,0]*np.pi/180)))
            #abs dif
            ph1.append(np.abs((bus.Vang_linopt[0,tsp].value-bus.Vang_NL[0,tsp]*np.pi/180)))
            ph2.append(np.abs((bus.Vang_linopt[1,tsp].value-(bus.Vang_NL[1,tsp]*-2)*np.pi/180)))
            ph3.append(np.abs((bus.Vang_linopt[2,tsp].value-bus.Vang_NL[2,tsp]*np.pi/180)))
            
        plt.plot(ph1,'ro', label='ph1')
        plt.plot(ph2,'go', label='ph2')
        plt.plot(ph3,'bo', label='ph3')
        plt.ylabel('Vang difference [rad]')
        plt.xlabel('Node')
        plt.ylim((-.1, 3))
        plt.legend()
        plt.show()

    return

def plot_bus(buskey):
    for ts in range(timesteps):
        import matplotlib.pyplot as plt
        # Plot lin result
        print('Linear sln')
        ph1 = []
        ph2 = []
        ph3 = []
        for key, bus in myfeeder.busdict.items():
            if bus.name == buskey:
        
                ph1.append(np.sqrt(bus.Vmagsq_linopt[0,ts].value))
                ph2.append(np.sqrt(bus.Vmagsq_linopt[1,ts].value))
                ph3.append(np.sqrt(bus.Vmagsq_linopt[2,ts].value))       
        
        plt.plot(ph1,'ro', label='ph1')
        plt.plot(ph2,'go', label='ph2')
        plt.plot(ph3,'bo', label='ph3')
        plt.ylabel('Vmag [p.u.]')
        plt.xlabel('Node')
        plt.ylim((0.8, 1.1))
        plt.legend()
        plt.show()
        
        print('Nonlinear sln')
        
        # Plot NL actuation result
        ph1 = list()
        ph2 = list()
        ph3 = list()
        for key, bus in myfeeder.busdict.items():
            if bus.name == busID:
                ph1.append(bus.Vmag_NL[0,ts]/(bus.kVbase_phg*1000))
                ph2.append(bus.Vmag_NL[1,ts]/(bus.kVbase_phg*1000))
                ph3.append(bus.Vmag_NL[2,ts]/(bus.kVbase_phg*1000))
        
        plt.plot(ph1,'ro', label='ph1')
        plt.plot(ph2,'go', label='ph2')
        plt.plot(ph3,'bo', label='ph3')
        plt.ylabel('Vmag [p.u.]')
        plt.xlabel('Node')
        plt.ylim((0.8, 1.1))
        plt.legend()
        plt.show()
        
    return

def export_gen():
    
    df = pd.DataFrame()
    for key1 in gendict:
        for key2 in gendict[key1]:
            for n, ph in enumerate(['a','b','c']):
                df[f'gen_{key1}_{key2}_{ph}'] = gendict[key1][key2][n]
    for key1 in shuntgendict:
        for key2 in shuntgendict[key1]:
            for n, ph in enumerate(['a','b','c']):
                df[f'shunt_{key1}_{key2}_{ph}'] = shuntgendict[key1][key2][n]
                
    df.to_csv(loadfolder+'export_gen/'+'gen.csv')
    
    return

def export_flow():
    
    dfp = pd.DataFrame()
    dfq = pd.DataFrame()
    for key1 in flowdict:
        for key2 in flowdict[key1]:
            for n, ph in enumerate(['a','b','c']):
                if key2 == 'P':
                    dfp[f'{key1}_{key2}_{ph}'] = flowdict[key1][key2][n]
                if key2 == 'Q':
                    dfq[f'{key1}_{key2}_{ph}'] = flowdict[key1][key2][n]                  
    df = pd.concat([dfp,dfq],axis=1)
    
    df.to_csv(loadfolder+'results_flow_actcost/'+'flow.csv')           
    #dfp.to_csv(filepath+'results_flow_actcost/'+'pflow.csv')
    #dfq.to_csv(filepath+'results_flow_actcost/'+'qflow.csv')
    
    return

def sum_flows():
    pflow = []
    qflow = []
# =============================================================================
#     for key1, val1 in flowdict.items():
#         for key2, val2 in val1.items():
#             if key2 == P:
# =============================================================================
    for key in flowdict:
        pflow.append(flowdict[key]['P'])
        qflow.append(flowdict[key]['Q'])
    print('excluding load:')
    print(f'Pnet: {sum(pflow)}, Qnet: {sum(qflow)}')
    print()
      
    
# =============================================================================
#     pld = []
#     qld = []
#     
#     for key, iload in myfeeder.loaddict.items():
#         pld.append(iload.Psched)
#         qld.append(iload.Qsched)
#     print('sum loads:')
#     print(f'Pnet_load: {sum(pld)}, Qnet_load: {sum(qld)}')
#     print()
#     
#     print(f'Pnet: {sum(pld)+sum(pflow)}, Qnet: {sum(qld)+sum(qflow)}')
# =============================================================================
    
    return

def sum_gen():
    
    for key, inode in myfeeder.busdict.items():
        if inode.type == 'SLACK' or inode.type == 'Slack' or inode.type == 'slack':
            conn = inode.edges_out
            print(conn)
            break
    Pout, Qout = 0, 0
    for iline in conn:
        Pout += np.array(iline.P_linopt.value)*subkVAbase
        Qout += np.array(iline.Q_linopt.value)*subkVAbase
    print('feeder head power flows:')
    print(f"P: {Pout}")
    print(f"Q: {Qout}")
    print()
    pld = []
    qld = []
    
    for key, iload in myfeeder.loaddict.items():
        pld.append(iload.Psched)
        qld.append(iload.Qsched)
        
    pgen = []
    qgen = []
    for key in gendict:
        pgen.append(gendict[key]['P'])
        qgen.append(gendict[key]['Q'])
    for key, ibus in myfeeder.busdict.items():
        if len(ibus.cap) > 0:
            qgen.append(ibus.cap[0].Qvec)
        
    print('net injections discluding slack:')
    print(f'Pnet: {sum(pld)+sum(pgen)}, Qnet: {sum(qld)+sum(qgen)}')
    print()
    print('sum acutators:')
    print(f'Pnet: {sum(pgen)}, Qnet: {sum(qgen)}')
    print()
    print('sum load:')
    print(f'Pnet: {sum(pld)}, Qnet: {sum(qld)}')
    

    return

def ploto1():
    
    Vmag_match = [0.995]*3#[1, 1, 1]
    Vang_match = [0 - np.radians(1), 4/3*np.pi - np.radians(1), 2/3*np.pi - np.radians(1)] 
    pltdict = {}   
    for key, bus in myfeeder.busdict.items():
        if key == '680':
            pltdict[key] = {
                    'vmag': {'1':bus.Vmag_NL[0,:]/(bus.kVbase_phg*1000),
                             '2':bus.Vmag_NL[1,:]/(bus.kVbase_phg*1000),
                             '3':bus.Vmag_NL[2,:]/(bus.kVbase_phg*1000)},
                    'vang': {'1': bus.Vang_NL[0,:]*np.pi/180,
                             '2': (bus.Vang_NL[1,:]+np.array([360]*timesteps))*np.pi/180,
                             '3': bus.Vang_NL[2,:]*np.pi/180}}
                    
            plt.plot(pltdict[key]['vmag']['1'],'ro', label=f'{key}_ph1')
            plt.plot(pltdict[key]['vmag']['2'],'go', label=f'{key}_ph2')
            plt.plot(pltdict[key]['vmag']['3'],'bo', label=f'{key}_ph3')
            plt.axhline(y=Vmag_match[0], color='r', linestyle='-')
            plt.ylabel('Vmag [p.u.]')
            plt.xlabel('timestep')
            plt.xticks(np.arange(0,22,2))
            plt.ylim((0.95, 1.05))
            plt.legend()
            plt.show()
            
            plt.plot(pltdict[key]['vang']['1'],'ro', label=f'{key}_ph1')
            plt.plot(pltdict[key]['vang']['2'],'go', label=f'{key}_ph2')
            plt.plot(pltdict[key]['vang']['3'],'bo', label=f'{key}_ph3')
            plt.axhline(y=Vang_match[0], color='r', linestyle='-')
            plt.axhline(y=Vang_match[1], color='g', linestyle='-')
            plt.axhline(y=Vang_match[2], color='b', linestyle='-')
            plt.ylabel('Vang [rad]')
            plt.xlabel('timestep')
            plt.xticks(np.arange(0,22,2))
            plt.ylim((-1, 6))
            plt.legend()
            plt.show()
    
    
    return

def ploto2():
    
    
    return

def ploto3(nodes):
    
    keyls = []
    pltdict = {}
    for key, bus in myfeeder.busdict.items():
        keyls.append(key)
    for key, bus in myfeeder.busdict.items():
        pltdict[key] = {
                'vmag': {'1':bus.Vmag_NL[0,:]/(bus.kVbase_phg*1000),
                         '2':bus.Vmag_NL[1,:]/(bus.kVbase_phg*1000),
                         '3':bus.Vmag_NL[2,:]/(bus.kVbase_phg*1000)},
                'vang': {'1': bus.Vang_NL[0,0]*np.pi/180,
                         '2': bus.Vang_NL[1,0]*np.pi/180,
                         '3': bus.Vang_NL[2,0]*np.pi/180,}          
                }

    print()   
    print('Vmag nonlinear')
    for node in nodes:  
        plt.plot(pltdict[keyls[node]]['vmag']['1'],'ro', label=f'{keyls[node]}_ph1')
        plt.plot(pltdict[keyls[node]]['vmag']['2'],'go', label=f'{keyls[node]}_ph2')
        plt.plot(pltdict[keyls[node]]['vmag']['3'],'bo', label=f'{keyls[node]}_ph3')
        plt.ylabel('Vmag [p.u.]')
        plt.xlabel('timestep')
        plt.xticks(np.arange(0,22,2))
        plt.ylim((0.97, 1.07))
        plt.legend()
        plt.show()
    
    return

def plotpf():

    '# ~~~ INPUT NODE ~~~ #'  
    nodeloc = 83 # [INPUT HERE]
    '______________________'
    
    dfdtarg = pd.read_csv(genpath, index_col = 0)
    dfdtarg.index += 1
    Pstr, Qstr, Pkey, Qkey = f'{nodeloc}_P', f'{nodeloc}_Q', [], []
    for head in dfdtarg.columns:
        if Pstr in head:
            Pkey.append(head)
        if Qstr in head:
            Qkey.append(head)
    netpflow_targ = np.transpose(dfdtarg[Pkey].values/subkVAbase)
    netqflow_targ = np.transpose(dfdtarg[Qkey].values/subkVAbase)
    
    return

    
def print_pflow():
    np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})
    for key, line in myfeeder.linedict.items():
        P,Q = line.P_linopt.value[0][0]*subkVAbase//1, line.Q_linopt.value[0][0]*subkVAbase//1
        if line.MVArating_3ph != complex(0):
            capacity = line.MVArating_3ph*1000/3//1
            if np.abs(P)+np.abs(Q) <= capacity:
                print(f'{key} - kVA rating: {capacity}')
            else:
                print(f'{key} - WARNING - kVA rating: {capacity}, S = {np.sqrt(P**2+Q**2)//1} ({np.round(np.sqrt(P**2+Q**2)/capacity*100,1)}%)')
            print(f'P: {P}, Q: {Q}, |P|+|Q| = {np.abs(P)+np.abs(Q)} ({np.round((np.abs(P)+np.abs(Q))/capacity*100,1)}%)')
            print('')
        else:
            print(f'{key} - kVA rating: N/A')
            print(f'P: {P}, Q: {Q}, |P|+|Q| = {np.abs(P)+np.abs(Q)}')
    return
    
def print_shunt(ts=0, ph=0):
    for key, icap in myfeeder.shuntdict.items():
        if icap.id[0:8] == 'sync_cap':
            print(key,'(sync): max = ', round(-icap.Qvec[ph][ts],2))
            print(round(icap.Qgen.value[ph][ts]*subkVAbase,2))
        if icap.id[0:3] == 'cap':
            print(key,'(cap): ', round(-icap.Qvec[ph][ts],2))
    return
        
def print_inj(ts=0, ph=0):
    Pld_all = []
    Qld_all = []
    for key, iload in myfeeder.loaddict.items():
        print(f'{key} - P_ld: {iload.Psched[ph][ts]//1}, Q_ld: {iload.Qsched[ph][ts]//1}')
        #for key2, iact in myfeeder.actdict.items():
            #if key2 == key:
        Pld_all.append(iload.Psched[ph][ts])
        Qld_all.append(iload.Qsched[ph][ts])
    print(f'Total P: {np.sum(Pld_all)}, Total Q: {np.sum(Qld_all)}')
    return

def print_V(ts=1, ph = 0):
    print(f'ts = {ts}, ph = {ph}')
    for key, bus in myfeeder.busdict.items():
        print(f'{key} -   V_NL = {np.round((bus.Vmag_NL/bus.kVbase_phg/1000)[ph,ts],4)}, Vlin = {np.round(np.sqrt(bus.Vmagsq_linopt.value[ph,ts]),4)}')
    return

def analysis(ts = 1):
    np.set_printoptions(formatter={'float': lambda x: "{0:0.1f}".format(x)})
    print('')
    print('~~~~~  POWER FLOWS  ~~~~~')
    print('')
    for key, line in myfeeder.linedict.items():
        if line.MVArating_3ph != complex(0):
            capacity = line.MVArating_3ph*1000/3//1
            S = np.round((line.Imag_NL*line.from_node.kVbase_phg)[:,ts],2)
            print(f'{key} - kVA rating: {capacity}')
            print(f'        |S| = {S} ({np.round(S/capacity*100,1)}%)')
        else:
            S = np.round((line.Imag_NL*line.from_node.kVbase_phg)[:,ts],2)
            print(f'{key} - kVA rating: N/A')
            print(f'        |S| = {S}')
    print('')
    np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})
    print('~~~~~  VOLTAGES  ~~~~~')
    print('')
    for key, bus in myfeeder.busdict.items():
        print(f'{key} -   V = {np.round((bus.Vmag_NL/bus.kVbase_phg/1000)[:,ts],4)}, {bus.Vang_NL[:,ts]}')#{np.round(bus.Vang_NL[:,ts]/1,5)}')
    
    return

def print_zeroV(ts=1):
    for key, ibus in myfeeder.busdict.items():
        if (ibus.Vmag_NL[:,ts] == np.zeros(3)).all():
            print(key)
    return

def print_zeroP_linopt(ts=1):
    temp = []
    for key, line in myfeeder.linedict.items():
        if line.P_linopt.value == None:
            temp.append(key)
            print(temp)
            print(len(temp))
    return
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            