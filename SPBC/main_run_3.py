# THIS IS THE INTERFACE FOR THE S-PBC
#SPBC-HIL files moved 7/15/19

# In[2]:
    
from setup_3 import *
from constraints_3 import *
from dss_3 import *

    # In[3]:
    
    ## WORKSPACE: CURRENT MODEL FOR TARGET GENERATION ###
def spbc_run(act_node1,act_node2,act_node3,act_node4): #write 'none' if doesnt exist    
    # Enter the path/name of the model's excel file and import
    
    #Test Case
    filepath = "/Users/jasperpakshong/Documents/Berkeley/ENERGISE/Test_Cases/IEEE13_bal/"
    modelpath = filepath + "016 GB_IEEE13_balance all ver2.xls"
    
    modeldata = pd.ExcelFile(modelpath)
     
    # All GridBright load files should be in the following folder
    #loadfolder = "/Users/jasperpakshong/Documents/Berkeley/ENERGISE/IEEE13/"
    #loadpath = loadfolder + "IEEE13testload_w_extreme_act.xlsx"
    
    #Test Case
    loadfolder = "/Users/jasperpakshong/Documents/Berkeley/ENERGISE/Test_Cases/IEEE13_bal/"
    loadpath = loadfolder + "016 GB_IEEE13_balance_sigBuilder_Q_12_13_norm03_act.xlsx"
    #001_phasor08_IEEE13_time_sigBuilder_1300-1400_norm03_3_1.xlsx
    
    actpath = loadpath
    
    # Enter name of actuator costs file - should be in same folder as the load file
    costfn_on_off = 0 # 0=off, 1=on
    if costfn_on_off == 1:
        actcostpath = loadfolder + 'act_costs_2_1_try2.xlsx'
        actcostdata = pd.read_excel(actcostpath, index_col=0)
     
    # Specify substation kV, kVA bases, and the number of timesteps in the load data
    subkVbase_phg = 4.16/np.sqrt(3)
    subkVAbase = 5000
    timesteps = 3
    
    # Create feeder object
    myfeeder = feeder(modelpath,loadfolder,loadpath,actpath,timesteps,subkVbase_phg,subkVAbase)
     
    myfeeder
    
    # In[6]:
    
    # Run optimization problem and generate targets
    
    # set tuning parameters: 0 = off. lam1 - phasor target, lam2 - phase balancing, lam3 - voltage volatility
    lam1 = 0
    lam2 = 0
    lam3 = 1
    
    obj = 0
    #pdb.set_trace()
    for ts in range(0,myfeeder.timesteps):
        for key, bus in myfeeder.busdict.items():
        #for key, bus in myfeeder.busdict['633']:
            #print(bus.name)
            
    # objective 1 - phasor target  
    
            if bus.name == 'bus_675':
                Vmag_match = .98
                Vang_a_match = 0
                Vang_b_match = 4/3*2*np.pi
                Vang_c_match = 2/3*np.pi
                #pdb.set_trace()
                #normalize to 2pi rad by adjusting to 2Pi / 2pi
                if (bus.phasevec == np.ones((3,timesteps))).all():
                    obj = obj + lam1*((cp.square(bus.Vang_linopt[0,ts]-Vang_a_match) + cp.square(bus.Vang_linopt[1,ts]-Vang_b_match) + cp.square(bus.Vang_linopt[2,ts]-Vang_c_match)))
                    obj = obj + lam1*((cp.square(bus.Vmagsq_linopt[0,ts]-Vmag_match) + cp.square(bus.Vmagsq_linopt[1,ts]-Vmag_match) + cp.square(bus.Vmagsq_linopt[2,ts]-Vmag_match)))
    
            
    # objective 2 - phase balancing
            if (bus.phasevec == np.array([[1],[1],[0]])).all():
                obj = obj + lam2*cp.square(bus.Vmagsq_linopt[0,ts]-bus.Vmagsq_linopt[1,ts])
            if (bus.phasevec == np.array([[1],[0],[1]])).all():
                obj = obj + lam2*cp.square(bus.Vmagsq_linopt[0,ts]-bus.Vmagsq_linopt[2,ts])
            if (bus.phasevec == np.array([[0],[1],[1]])).all():
                obj = obj + lam2*cp.square(bus.Vmagsq_linopt[1,ts]-bus.Vmagsq_linopt[2,ts])
            if (bus.phasevec == np.ones((3,timesteps))).all():
                obj = obj + lam2*(cp.square(bus.Vmagsq_linopt[0,ts]-bus.Vmagsq_linopt[1,ts]) + cp.square(bus.Vmagsq_linopt[0,ts]-bus.Vmagsq_linopt[2,ts]) + cp.square(bus.Vmagsq_linopt[1,ts]-bus.Vmagsq_linopt[2,ts]))
    
    # objective 3.1 - power flow
                '''
        for key, line in myfeeder.linedict.items():
            if line.name == 'line_651to632':
                obj += lam3*cp.square(line.P_linopt[0,ts])
                obj += lam3*cp.square(line.P_linopt[1,ts])
                obj += lam3*cp.square(line.P_linopt[2,ts])
                obj += lam3*cp.square(line.Q_linopt[0,ts])
                obj += lam3*cp.square(line.Q_linopt[1,ts])
                obj += lam3*cp.square(line.Q_linopt[2,ts])
                '''
    
    # objective 3.2 - voltage volitility
    for ts in range(1,myfeeder.timesteps):
        for key, bus in myfeeder.busdict.items():
            if bus.name == 'bus_632':
                obj += lam3*cp.square(bus.Vmagsq_linopt[0,ts]-bus.Vmagsq_linopt[0,ts-1]+
                                      bus.Vmagsq_linopt[1,ts]-bus.Vmagsq_linopt[1,ts-1]+
                                      bus.Vmagsq_linopt[2,ts]-bus.Vmagsq_linopt[2,ts-1])
                
    # add cost function to actuators       
    if costfn_on_off == 1:
        for ts in range(0,myfeeder.timesteps):
            '''
            for key,iact in myfeeder.actdict.items():
                #print(key)
                #print(iact.Pgen[:,timesteps-1].value)
                key_str = str(key)
                #if iact.Pgen[:,timesteps-1].value is not None:
                #obj += cp.square(iact.Pgen[:,ts] * actcostdata[key_str][ts])
                for idx in range(0,3):
                    obj += cp.square(iact.Pgen[idx,ts:ts+1] * actcostdata[key_str][ts])
            '''    
                    
            for key, inode in myfeeder.busdict.items():
                for iact in inode.actuators:
                    key_str = str(key)
                    for idx in range(0,3):
                        obj += cp.square(iact.Pgen[idx,ts:ts+1] * actcostdata[key_str][ts])
                    
     
    objective = cp.Minimize(obj) 
    
    constraints = cvx_set_constraints(myfeeder,1) # Second argument turns actuators on/off
    prob = cp.Problem(objective, constraints)
    result = prob.solve(verbose=False,eps_rel=1e-3,eps_abs=1e-3)
    
    # In[7]:
    
    DSS_alltimesteps(myfeeder,1) # Second argument turns voltage alarms on/off
    
    #export_Vtargets(myfeeder)
    ##[jasper] - fn to get target in vector format for LPBC
    #nodes_arr,Vmag_targ,Vang_targ,KVbase = get_targets(myfeeder)
    
    # Vtargdict[key(nodeID)][Vmag/Vang/KVbase]
    Vtargdict, act_keys = get_targets(myfeeder)
    return Vtargdict, act_keys

# In[8]:
# Run main_run
Vtargdict, act_keys = spbc_run(0,0,0,0)

# In[9]:
# JASPER temp
#for key,iact in myfeeder.actdict.items():
'''    
for key, inode in myfeeder.busdict.items():
    for iact in inode.actuators:

        print('P:',iact.Pgen.value)
        #print('Q:',iact.Qgen.value)

        for ts in range(0,myfeeder.timesteps):
            print(key)
            print(iact.Pgen[:,ts].value)
            print('normalized')
            print(iact.Pgen[:,ts].value/(iact.Psched[:,ts]/inode.kVAbase))
            print('Psched')
            print(iact.Psched[:,ts]/inode.kVAbase)

for ts in range(0,myfeeder.timesteps):
    for key, line in myfeeder.linedict.items():
# objective 3 - voltage volitility
        #if choose_obj == 3:
            if line.name == 'line_651to632':
                print(np.max(line.P_linopt[:,:].value))
'''

# Plot first timestep of result

tsp = 0 # select timestep for convergence plot

# DSS_alltimesteps(myfeeder,1) 
plot = 0
if plot == 1:
    # Plot lin result
    print('Linear sln')
    ph1 = []
    ph2 = []
    ph3 = []
    for key, bus in myfeeder.busdict.items():
    
        ph1.append(np.sqrt(bus.Vmagsq_linopt[0,0].value))
        ph2.append(np.sqrt(bus.Vmagsq_linopt[1,0].value))
        ph3.append(np.sqrt(bus.Vmagsq_linopt[2,0].value))
    
    
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
    plt.legend()
    plt.show()
    
    print('Vang linear')
    
    ph1 = list()
    ph2 = list()
    ph3 = list()
    for key, bus in myfeeder.busdict.items():
        ph1.append(bus.Vang_linopt[0,0].value)
        ph2.append(bus.Vang_linopt[1,0].value)
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
        ph1.append(bus.Vang_NL[0,0]*np.pi/180)
        ph2.append((bus.Vang_NL[1,0]*-2)*np.pi/180)
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
    
        ph1.append(np.abs((np.sqrt(bus.Vmagsq_linopt[0,tsp].value)-bus.Vmag_NL[0,tsp]/(bus.kVbase_phg*1000))/(bus.Vmag_NL[0,tsp]/(bus.kVbase_phg*1000))))
        ph2.append(np.abs((np.sqrt(bus.Vmagsq_linopt[1,tsp].value)-bus.Vmag_NL[1,tsp]/(bus.kVbase_phg*1000))/(bus.Vmag_NL[1,tsp]/(bus.kVbase_phg*1000))))
        ph3.append(np.abs((np.sqrt(bus.Vmagsq_linopt[2,tsp].value)-bus.Vmag_NL[2,tsp]/(bus.kVbase_phg*1000))/(bus.Vmag_NL[2,tsp]/(bus.kVbase_phg*1000))))
    
    plt.plot(ph1,'ro', label='ph1')
    plt.plot(ph2,'go', label='ph2')
    plt.plot(ph3,'bo', label='ph3')
    plt.ylabel('Vmag difference [%]')
    plt.xlabel('Node')
    plt.ylim((-.01, .1))
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
    plt.ylabel('Vang difference [%]')
    plt.xlabel('Node')
    plt.ylim((-.01, .1))
    plt.legend()
    plt.show()
