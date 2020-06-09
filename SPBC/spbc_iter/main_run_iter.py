from setup_3 import *
from constraints_3 import *
from dss_3 import *
import datetime
import time
import cvxpy as cp


def feeder_init(modelpath, loadfolder, loadpath, timesteps, timestepcur, subkVbase_phg, subkVAbase, PVforecast, act_init, Psat_nodes=[],Qsat_nodes=[]):
    modeldata = pd.ExcelFile(modelpath)
    actpath = loadpath
    
    # set dummy values for undefined variables
    date = datetime.datetime.now()
    month = date.month
    day = date.day
    hour = date.hour
    minute = date.minute

    refphasor = np.ones((3,2))
    refphasor[:,0]=1
    refphasor[:,1]=[0,4*np.pi/3,2*np.pi/3]

    # #get feeder
    myfeeder = feeder(modelpath,loadfolder,loadpath,actpath,timesteps,timestepcur,
                      subkVbase_phg,subkVAbase,refphasor,Psat_nodes,Qsat_nodes,PVforecast, act_init)
    myfeeder
    
    phase_size = 0
    for key, inode in myfeeder.busdict.items():
        if inode.type == 'SLACK' or inode.type == 'Slack' or inode.type == 'slack':
            for ph in inode.phases:
                phase_size += 1
    
    return phase_size, myfeeder


def lin_optimization(myfeeder, timesteps, enable_actuators, verbose=True,eps_rel=1e-5,eps_abs=1e-10, max_iter = 50000): #write 'none' if doesnt exist    
    # set tuning parameters of objectives: 0 = off
    # lam1 - phasor target, lam2 - phase balancing, lam3 - voltage volatility
    # lamcost - const function, lamdistpf - transmission/distribution power flow
    lam1 = 1
    lam2 = 0
    lam3 = 0
    lam4 = 0
    lam_kyle = 0
    
    lamcost = 1
    lamdistpf = 50
    
    # turn additional objectives on/off
    costfn_on_off = 0 # 0=off, 1=on  
    distpf_on_off = 0 # 0=off, 1=on
    
    #  phasor target settings:
    if lam1 > 0:
        target_key = '18'
        # Vmag_match = [.985]*3
        # Vang_match = [0 - np.radians(1),
        #  4/3*np.pi - np.radians(1), 2/3*np.pi - np.radians(1)] 

        Vmag_match = [0.99]*3
        Vang_match = [0 + np.radians(-1), 4/3*np.pi + np.radians(-1), 2/3*np.pi + np.radians(-1)] 
    
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
                    
                    #pdb.set_trace()
                    #normalize to 2pi rad by adjusting to 2Pi / 2pi
                    if (bus.phasevec == np.ones((3,timesteps))).all():
                        obj = obj + lam1*((cp.square(bus.Vang_linopt[0,ts]-Vang_match[0]) + cp.square(bus.Vang_linopt[1,ts]-Vang_match[1]) + cp.square(bus.Vang_linopt[2,ts]-Vang_match[2])))
                        obj += lam1*((cp.square(bus.Vmagsq_linopt[0,ts]-Vmag_match[0]**2) + cp.square(bus.Vmagsq_linopt[1,ts]-Vmag_match[1]**2) + cp.square(bus.Vmagsq_linopt[2,ts]-Vmag_match[2]**2)))


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


    # kyle's balancing objective
    if lam_kyle > 0:
        for ts in range(0,myfeeder.timesteps):
            for key, bus in myfeeder.busdict.items():
                if (bus.phasevec == np.array([[1],[1],[0]])).all():
                    obj = obj + lam_kyle*cp.square(bus.Vmagsq_linopt[0,ts]-bus.Vmagsq_linopt[1,ts])
                if (bus.phasevec == np.array([[1],[0],[1]])).all():
                    obj = obj + lam_kyle*cp.square(bus.Vmagsq_linopt[0,ts]-bus.Vmagsq_linopt[2,ts])
                if (bus.phasevec == np.array([[0],[1],[1]])).all():
                    obj = obj + lam_kyle*cp.square(bus.Vmagsq_linopt[1,ts]-bus.Vmagsq_linopt[2,ts])
                if (bus.phasevec == np.ones((3,timesteps))).all():
                    obj = obj + lam_kyle*cp.square(bus.Vmagsq_linopt[0,ts]-bus.Vmagsq_linopt[1,ts]) + cp.square(bus.Vmagsq_linopt[0,ts]-bus.Vmagsq_linopt[2,ts]) + cp.square(bus.Vmagsq_linopt[1,ts]-bus.Vmagsq_linopt[2,ts])
                
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

    objective = cp.Minimize(obj)
    constraints = cvx_set_constraints(myfeeder,enable_actuators) # Second argument turns actuators on/off
    prob = cp.Problem(objective, constraints)
    #result = prob.solve()  
    result = prob.solve(verbose=verbose,eps_rel=eps_rel,eps_abs=eps_abs, max_iter = 50000)

    Vtargdict, act_keys = get_targets(myfeeder)

    return prob, result, objective, Vtargdict, act_keys


# refphasor,Psat_nodes,Qsat_nodes,perf_nodes,timestepcur (inputs from old spbc_run)
def spbc_iter_run(timestepcur):

    'IEEE13_UNBALANCED'
    # filepath = "IEEE13/"
    # modelpath = filepath + "001 phasor08_IEEE13_OPAL.xls"
    # loadfolder = "IEEE13/"
    # loadpath = loadfolder + "001_phasor08_IEEE13_T12-3.xlsx"

    'IEEE13_BALANCED'
    # filepath = "IEEE13_bal/"
    # modelpath = filepath + "016_GB_IEEE13_balance_reform.xlsx"
    # loadfolder = "IEEE13_bal/"
    # loadpath = loadfolder + "016_GB_IEEE13_balance_norm03.xlsx"

    'UCB33'
    filepath = "UCB33/"
    modelpath = filepath + "005_GB_UCB33_opal_v3.xlsx"
    loadfolder = "UCB33/"
    loadpath = loadfolder + "005_GB_UCB33_time_sigBuilder_Q_13_14_norm03.xlsx"

    'PL0001_v2'
    # filepath = "PL0001_v2/"
    # modelpath = filepath + "PL0001_OPAL_working_reform_xfmr.xlsx"
    # #modelpath = filepath + "PL0001_OPAL_working_reform_notrans_F.xlsx"
    # loadfolder = "PL0001_v2/"
    # loadpath = loadfolder + "PL0001_July_Q_F.xlsx"

    # filepath = "IEEE test cases/Thirteenbus/"
    # modelpath = filepath + "IEEE13_model_M_confirm.xls"
    # modeldata = pd.ExcelFile(modelpath)
     
    # # All GridBright load files should be in the following folder
    # loadfolder = "IEEE test cases/Thirteenbus/"
    # loadpath = loadfolder + "IEEE13_load_M_modified.xlsx"
    # actpath = loadpath

     
    # Specify substation kV, kVA bases, and the number of timesteps in the load data
    'IEEE13'
    # subkVbase_phg = 4.16/np.sqrt(3)
    # subkVAbase = 5000.
    # timesteps = 1

    'UCB33'
    subkVbase_phg = 12.47/np.sqrt(3)
    subkVAbase = 3000.
    timesteps = 1

    'PL0001'
    # subkVbase_phg = 12.6/np.sqrt(3)
    # subkVAbase = 1500.
    # timesteps = 1


    # Specify initial timestep
    date = datetime.datetime.now()
    month = date.month
    day = date.day
    hour = date.hour
    minute = date.minute
    #timestepcur = hour*60+minute
    # timestepcur = 0 #8*60 # [INPUT HERE] Manual input of start time
    # for HIL timestepcur sent from wrapper

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

    act_init = {'18': {'a': 707, 'b': 707, 'c': 707},
        # '652': {'a': 200, 'b': 0, 'c':0},
        # '671': {'a': 100, 'b': 0, 'c':0},
        # '692': {'a': 200, 'b': 0, 'c':0}
            }

    phase_size, myfeeder = feeder_init(modelpath, loadfolder, loadpath, timesteps, timestepcur, subkVbase_phg, subkVAbase, PVforecast, act_init)


    num_iterations = 5
    enable_actuators = True
    bus_map = {}
    act_map = {}

    nbuses = 0
    for k, bus in myfeeder.busdict.items():
        bus_map[k] = nbuses
        nbuses += 1
    
    nacts = 0
    for k, bus in myfeeder.actdict.items():
        act_map[k] = nacts
        nacts += 1

    NLvslinear_mag = np.zeros([nbuses,3])
    NLvslinear_ang = np.zeros([nbuses,3])

    NLvoltage = np.zeros([nbuses,3])
    NLangle = np.zeros([nbuses,3])
    linvoltage = np.zeros([nbuses,3])
    linangle = np.zeros([nbuses,3])
    Pdispatch = np.zeros([nacts,3])
    Qdispatch = np.zeros([nacts,3])

    mag_errors_pu = []
    ang_errors = []
    obj_vals = []

    for i in range(num_iterations):
        prob, result, objective, Vtargdict, act_keys = lin_optimization(myfeeder, timesteps, enable_actuators, verbose=False)
        DSS_alltimesteps(myfeeder,0)

        #assumes keys in busdict go from 0 -> n-1
        for k, bus in myfeeder.busdict.items():

            idx = bus_map[k]
            NLvoltage[idx,:] = bus.Vmag_NLpu.T
            NLangle[idx,:] = bus.Vang_NL.T

            linvoltage[idx,:] = np.sqrt(bus.Vmagsq_linopt.value.T)
            linangle[idx,:] = np.degrees(bus.Vang_linopt.value.T)
            
        for k, act in myfeeder.actdict.items():
            idx = act_map[k]
            Pdispatch[idx,:] = act.Pgen.value.T
            Qdispatch[idx,:] = act.Qgen.value.T

        # not accounting for caps yet
        NLvslinear_mag = np.abs(NLvoltage - linvoltage)
        NLvslinear_ang = np.minimum(np.abs(NLangle - linangle),\
                                    np.abs(NLangle%360 - linangle),\
                                    np.abs(NLangle - linangle%360))


        obj_vals.append(objective.value)
        mag_errors_pu.append(np.average(NLvslinear_mag))
        ang_errors.append(np.average(NLvslinear_ang))

        print("ITERATION " + str(i) + "\n\n")
        print("Difference between linear and nonlinear voltages\n\n")
        print("maximum = " + str(np.amax(NLvslinear_mag)) + "V")
        print("average = " + str(np.average(NLvslinear_mag)) + "V")
        print("\n\n")
        print("Difference between linear and nonlinear angles\n\n")
        print("maximum = " + str(np.amax(NLvslinear_ang)) + " deg")
        print("average = " + str(np.average(NLvslinear_ang)) + " deg")
        print("\n\n")

    # # plot error curves
    # plot_subject = "Modified 13 node"
    # # plot_subject = "14 node"
    # plot_objective = "Phasor target 632 nominal constrained"
    
    # plt.plot(list(range(num_iterations)), mag_errors_pu)
    # plt.xlabel("Iterations")
    # plt.ylabel("Avg voltage magnitude error (p.u.)")
    # plt.title(plot_subject + " with " + plot_objective)
    # plt.xticks(list(range(num_iterations)))
    # plt.tight_layout()
    # plt.savefig("plots/" + plot_subject+"_"+plot_objective+"_Vmag.png")
    # plt.show()

    # plt.plot(list(range(num_iterations)), ang_errors)
    # plt.xlabel("Iterations")
    # plt.ylabel("Avg voltage angle error (deg)")
    # plt.title(plot_subject + " with " + plot_objective)
    # plt.xticks(list(range(num_iterations)))
    # plt.tight_layout()
    # plt.savefig("plots/" + plot_subject+"_"+plot_objective+"_Vang.png")
    # plt.show()

    # plt.plot(list(range(num_iterations)), obj_vals)
    # plt.xlabel("Iterations")
    # plt.ylabel("Optimzation objective value")
    # plt.title(plot_subject + " with " + plot_objective)
    # plt.xticks(list(range(num_iterations)))
    # plt.tight_layout()
    # plt.savefig("plots/" + plot_subject+"_"+plot_objective+"_obj.png")
    # plt.show()

    return subkVAbase, myfeeder, Vtargdict, act_keys

subkVAbase, myfeeder, Vtargdict, act_keys = spbc_iter_run(0)
    