# USES DSS TO SOLVE NONLINEAR POWER FLOW FOR A FEEDER OBJECT DEFINED IN THE 'SETUP' CODE
#SPBC-HIL files moved 7/15/19

# In[35]:

from setup_3 import *
import opendssdirect as dss
from dss_3 import *
import os


# In[36]:


# HELPER FUNCTIONS


# In[37]:


def DSS_loads(feeder,timestep):
# Uses DSS commands to add loads to a circuit
    for key,iload in feeder.loaddict.items():
        Pvec = iload.Psched[:,timestep]
        Qvec = iload.Qsched[:,timestep]
        for idx in range(0,3):
            # Constant P,Q = model 1
            if iload.phasevec[0,0] == 1:
                dss.run_command("New Load." + iload.name +"aCP Bus1=" + iload.node.name + ".1 Phases=1 Conn=Wye Model=1 kV=" 
                                + str(iload.node.kVbase_phg) + " kW=" + str(Pvec[0]*iload.constP)
                                + " kvar=" + str(Qvec[0]*iload.constP) + " Vminpu=0.8 Vmaxpu=1.2")
            if iload.phasevec[1,0] == 1:
                dss.run_command("New Load." + iload.name +"bCP Bus1=" + iload.node.name + ".2 Phases=1 Conn=Wye Model=1 kV=" 
                                + str(iload.node.kVbase_phg) + " kW=" + str(Pvec[1]*iload.constP)
                                + " kvar=" + str(Qvec[1]*iload.constP) + " Vminpu=0.8 Vmaxpu=1.2")
            if iload.phasevec[2,0] == 1:
                dss.run_command("New Load." + iload.name +"cCP Bus1=" + iload.node.name + ".3 Phases=1 Conn=Wye Model=1 kV=" 
                                + str(iload.node.kVbase_phg) + " kW=" + str(Pvec[2]*iload.constP)
                                + " kvar=" + str(Qvec[2]*iload.constP) + " Vminpu=0.8 Vmaxpu=1.2")
            
            # Constant Z = model 2
            if iload.phasevec[0,0] == 1:
                dss.run_command("New Load." + iload.name +"aCZ Bus1=" + iload.node.name + ".1 Phases=1 Conn=Wye Model=2 kV=" 
                                + str(iload.node.kVbase_phg) + " kW=" + str(Pvec[0]*iload.constZ)
                                + " kvar=" + str(Qvec[0]*iload.constZ) + " Vminpu=0.8 Vmaxpu=1.2")
            if iload.phasevec[1,0] == 1:
                dss.run_command("New Load." + iload.name +"bCZ Bus1=" + iload.node.name + ".2 Phases=1 Conn=Wye Model=2 kV=" 
                                + str(iload.node.kVbase_phg) + " kW=" + str(Pvec[1]*iload.constZ)
                                + " kvar=" + str(Qvec[1]*iload.constZ) + " Vminpu=0.8 Vmaxpu=1.2")
            if iload.phasevec[2,0] == 1:
                dss.run_command("New Load." + iload.name +"cCZ Bus1=" + iload.node.name + ".3 Phases=1 Conn=Wye Model=2 kV=" 
                                + str(iload.node.kVbase_phg) + " kW=" + str(Pvec[2]*iload.constZ)
                                + " kvar=" + str(Qvec[2]*iload.constZ) + " Vminpu=0.8 Vmaxpu=1.2")
            
            # Constant I = model 5
            if iload.phasevec[0,0] == 1:
                dss.run_command("New Load." + iload.name +"aCI Bus1=" + iload.node.name + ".1 Phases=1 Conn=Wye Model=5 kV=" 
                                + str(iload.node.kVbase_phg) + " kW=" + str(Pvec[0]*iload.constI)
                                + " kvar=" + str(Qvec[0]*iload.constI) + " Vminpu=0.8 Vmaxpu=1.2")
            if iload.phasevec[1,0] == 1:
                dss.run_command("New Load." + iload.name +"bCI Bus1=" + iload.node.name + ".2 Phases=1 Conn=Wye Model=5 kV=" 
                                + str(iload.node.kVbase_phg) + " kW=" + str(Pvec[1]*iload.constI)
                                + " kvar=" + str(Qvec[1]*iload.constI) + " Vminpu=0.8 Vmaxpu=1.2")
            if iload.phasevec[2,0] == 1:
                dss.run_command("New Load." + iload.name +"cCI Bus1=" + iload.node.name + ".3 Phases=1 Conn=Wye Model=5 kV=" 
                                + str(iload.node.kVbase_phg) + " kW=" + str(Pvec[2]*iload.constI)
                                + " kvar=" + str(Qvec[2]*iload.constI) + " Vminpu=0.8 Vmaxpu=1.2")
            
    return

def DSS_lines_1ph(iline,timestep):
# Helper function for DSS_lines
    Zmat = iline.R + 1j*iline.X
    itemindex = np.where(Zmat>=1e-7)
        
    r11 = iline.R[itemindex][0]
    x11 = iline.X[itemindex][0]
        
    phasestr = "."
    if iline.phasevec[0,0] == 1:
        phasestr = phasestr + "1."
    if iline.phasevec[1,0] == 1:
        phasestr = phasestr + "2."
    if iline.phasevec[2,0] == 1:
        phasestr = phasestr + "3."
    if phasestr[len(phasestr) - 1] == ".":
        phasestr = phasestr[:-1]

    dss.run_command("New Line." + iline.name +" Bus1=" + iline.from_node.name + phasestr + " Bus2=" 
                    + iline.to_node.name + phasestr + " BaseFreq=60 Phases=1"
                    + " rmatrix = (" + str(r11) + ")"
                    + " xmatrix = (" + str(x11) + ")")
    return
        
def DSS_lines_2ph(iline,timestep):
# Helper function for DSS_lines
    Zmat = iline.R + 1j*iline.X
        
    if Zmat[0][1] > 1e-7:
        Rtemp = iline.R[0:2][:,0:2]
        Xtemp = iline.X[0:2][:,0:2]
    elif Zmat[1][2] > 1e-7:
        Rtemp = iline.R[1:3][:,1:3]
        Xtemp = iline.X[1:3][:,1:3]
    elif Zmat[0][2] > 1e-7:
        Rtemp = np.array([[iline.R[0][0],iline.R[0][2]],[iline.R[2][0],iline.R[2][2]]])
        Xtemp = np.array([[iline.X[0][0],iline.X[0][2]],[iline.X[2][0],iline.X[2][2]]])
        
    r11 = Rtemp[0,0]
    r12 = Rtemp[1,0]
    r22 = Rtemp[1,1]
        
    x11 = Xtemp[0,0]
    x12 = Xtemp[1,0]
    x22 = Xtemp[1,1]
        
    phasestr = "."
    if iline.phasevec[0,0] == 1:
        phasestr = phasestr + "1."
    if iline.phasevec[1,0] == 1:
        phasestr = phasestr + "2."
    if iline.phasevec[2,0] == 1:
        phasestr = phasestr + "3."
    if phasestr[len(phasestr) - 1] == ".":
        phasestr = phasestr[:-1]
        
    dss.run_command("New Line." + iline.name +" Bus1=" + iline.from_node.name + phasestr + " Bus2=" 
                    + iline.to_node.name + phasestr + " BaseFreq=60 Phases=2"
                    + " rmatrix = (" + str(r11) + " | " + str(r12) + " " + str(r22) + ")"
                    + " xmatrix = (" + str(x11) + " | " + str(x12) + " " + str(x22) + ")")
    return


def DSS_lines_3ph(iline,timestep):
# Helper function for DSS_lines
    r11 = iline.R[0,0]
    r12 = iline.R[1,0]
    r22 = iline.R[1,1]
    r13 = iline.R[2,0]
    r23 = iline.R[2,1]
    r33 = iline.R[2,2]
        
    x11 = iline.X[0,0]
    x12 = iline.X[1,0]
    x22 = iline.X[1,1]
    x13 = iline.X[2,0]
    x23 = iline.X[2,1]
    x33 = iline.X[2,2]
        
    dss.run_command("New Line." + iline.name +" Bus1=" + iline.from_node.name + ".1.2.3 Bus2=" 
                    + iline.to_node.name + ".1.2.3 BaseFreq=60 Phases=3"
                    + " rmatrix = (" + str(r11) + " | " + str(r12) + " " + str(r22) + " | " + str(r13) + " " + str(r23) + " " + str(r33) + ")"
                    + " xmatrix = (" + str(x11) + " | " + str(x12) + " " + str(x22) + " | " + str(x13) + " " + str(x23) + " " + str(x33) + ")")
    return

def DSS_lines(feeder,timestep):
# Uses DSS commands to add lines to a circuit
    for key,iline in feeder.linedict.items():
        Zmat = iline.R + iline.X
        if np.sum(Zmat>1e-7) == 9 or np.sum(Zmat>1e-7) == 3:
            DSS_lines_3ph(iline,timestep)
        if np.sum(Zmat>1e-7) == 4 or np.sum(Zmat>1e-7) == 2:
            DSS_lines_2ph(iline,timestep)
        if np.sum(Zmat>1e-7) == 1:
            DSS_lines_1ph(iline,timestep)
    return

def DSS_caps(feeder,timestep):
# Uses DSS commands to add cap banks to a circuit
# Only set up to handle capacitors, no real power (real power would be an easy insert, though)
# Also assumes capacitors are either on or off throughout the entirety of a model's run (no switching back and forth)
    # Cap Qvec is read in with a negative sign, so this works out right
    for key,icap in feeder.shuntdict.items():
        if icap.id[0:3] == 'cap':
            kVentry = icap.node.kVbase_phg
            
            if icap.phasevec[0,0] == 1:
                dss.run_command("New Capacitor." + icap.name + "a Bus1=" + icap.node.name + ".1 phases=1 kVAR=" 
                                + str(-icap.Qvec[0,0]) + " kV=" + str(kVentry))
            if icap.phasevec[1,0] == 1:
                dss.run_command("New Capacitor." + icap.name + "b Bus1=" + icap.node.name + ".2 phases=1 kVAR=" 
                                + str(-icap.Qvec[1,0]) + " kV=" + str(kVentry))
            if icap.phasevec[2,0] == 1:
                dss.run_command("New Capacitor." + icap.name + "c Bus1=" + icap.node.name + ".3 phases=1 kVAR=" 
                                + str(-icap.Qvec[2,0]) + " kV=" + str(kVentry))
                
# 3/31/20 - added sync_cap to allow continuously variable Var support for transmission systems (synchronous condensors)
        # (+) Qgen is generating
        if icap.id[0:8] == 'sync_cap':
            kVentry = icap.node.kVbase_phg
            Qvec = icap.Qgen[:,timestep].value
            
            if icap.phasevec[0,0] == 1:
                dss.run_command("New Capacitor." + icap.name + "a Bus1=" + icap.node.name + ".1 phases=1 kVAR=" 
                                + str(Qvec[0]*icap.node.kVAbase) + " kV=" + str(kVentry))
            if icap.phasevec[1,0] == 1:
                dss.run_command("New Capacitor." + icap.name + "b Bus1=" + icap.node.name + ".2 phases=1 kVAR=" 
                                + str(Qvec[1]*icap.node.kVAbase) + " kV=" + str(kVentry))
            if icap.phasevec[2,0] == 1:
                dss.run_command("New Capacitor." + icap.name + "c Bus1=" + icap.node.name + ".3 phases=1 kVAR=" 
                                + str(Qvec[2]*icap.node.kVAbase) + " kV=" + str(kVentry))
        
    return

def DSS_switches(feeder,timestep):
# Uses DSS commands to add switches to a circuit
# There is no 'switch' object in DSS. Instead, all lines have built-in switches. So we just add a short line.
    for key,iswitch in feeder.switchdict.items():
        
        if iswitch.phasevec[0,0] == 1:
            dss.run_command("New Line." + iswitch.name + "a Bus1=" + iswitch.from_node.name + ".1 Bus2="
                            + iswitch.to_node.name 
                            + ".1 Phases=1 Switch=y r1=1e-4 r0=1e-4 x1=0.000 x0=0.000 c1=0.000 c0=0.000")
        if iswitch.phasevec[1,0] == 1:
            dss.run_command("New Line." + iswitch.name + "b Bus1=" + iswitch.from_node.name + ".2 Bus2="
                            + iswitch.to_node.name
                            + ".2 Phases=1 Switch=y r1=1e-4 r0=1e-4 x1=0.000 x0=0.000 c1=0.000 c0=0.000")
        if iswitch.phasevec[2,0] == 1:
            dss.run_command("New Line." + iswitch.name + "c Bus1=" + iswitch.from_node.name + ".3 Bus2="
                            + iswitch.to_node.name
                            + ".3 Phases=1 Switch=y r1=1e-4 r0=1e-4 x1=0.000 x0=0.000 c1=0.000 c0=0.000")
    return    

def DSS_trans(feeder,timestep):
# Uses DSS commands to add transformers to a circuit
# Note that the kV "base" being used here is ph-ph. This runs counter to how the official opendss manual...
# ... claims bases are defined for 1-ph transformers. I've tested it, and it appears the manual is mistaken.
    for key,itrans in feeder.transdict.items():
        
        if itrans.phasevec[0,0] == 1:
            dss.run_command("New Transformer." + itrans.name + "a Phases=1 Windings=2 buses=["
                            + itrans.w0_node.name + ".1.0, " + itrans.w1_node.name + ".1.0] conns =[wye, wye]"
                            + " kVs=[" + str(itrans.w0_kVbase_phg*np.sqrt(3)) + ", " + str(itrans.w1_kVbase_phg*np.sqrt(3)) + "]"
                            + " kVAs=[" + str(itrans.w0_kVAbase) + ", " + str(itrans.w1_kVAbase) + "]"
                            + " %Rs=[" + str(itrans.w0_rpu*100) + ", " + str(itrans.w1_rpu*100) + "]"
                            + " XHL=" + str(itrans.xpu*100)) #TODO: NOTE change this 100 to match Zbase ratio - come back to this (ask Keith)
            
        if itrans.phasevec[1,0] == 1:
            dss.run_command("New Transformer." + itrans.name + "b Phases=1 Windings=2 buses=["
                            + itrans.w0_node.name + ".2.0, " + itrans.w1_node.name + ".2.0] conns =[wye, wye]"
                            + " kVs=[" + str(itrans.w0_kVbase_phg*np.sqrt(3)) + ", " + str(itrans.w1_kVbase_phg*np.sqrt(3)) + "]"
                            + " kVAs=[" + str(itrans.w0_kVAbase) + ", " + str(itrans.w1_kVAbase) + "]"
                            + " %Rs=[" + str(itrans.w0_rpu*100) + ", " + str(itrans.w1_rpu*100) + "]"
                            + " XHL=" + str(itrans.xpu*100))

        if itrans.phasevec[2,0] == 1:
            dss.run_command("New Transformer." + itrans.name + "c Phases=1 Windings=2 buses=["
                            + itrans.w0_node.name + ".3.0, " + itrans.w1_node.name + ".3.0] conns =[wye, wye]"
                            + " kVs=[" + str(itrans.w0_kVbase_phg*np.sqrt(3)) + ", " + str(itrans.w1_kVbase_phg*np.sqrt(3)) + "]"
                            + " kVAs=[" + str(itrans.w0_kVAbase) + ", " + str(itrans.w1_kVAbase) + "]"
                            + " %Rs=[" + str(itrans.w0_rpu*100) + ", " + str(itrans.w1_rpu*100) + "]"
                            + " XHL=" + str(itrans.xpu*100))
    return

def DSS_actuators(feeder,timestep, verbose=False):
# Uses DSS commands to add the previously solved-for values of actuator dispatch to the model as negative loads.
    for key,iact in feeder.actdict.items():
        if verbose:
            print(f'{iact.name}_Pgen: {iact.Pgen[:,timestep].value*5000}') 
            print(f'{iact.name}_Qgen: {iact.Qgen[:,timestep].value*5000}')
        Pvec = -iact.Pgen[:,timestep].value #Pgen (+) = producing -> neg Pvec = producing
        #print(f'P: {np.array(Pvec)*100000}') #jasper TEMP
        
        Qvec = -iact.Qgen[:,timestep].value #Qgen (+) = producing -> neg Qvec = producing
        #print(f'Q: {np.array(Qvec)*100000}') #jasper TEMP
        for idx in range(0,3):
            if iact.phasevec[0,0] == 1:
                dss.run_command("New Load." + iact.name +"a Bus1=" + iact.node.name + ".1 Phases=1 Conn=Wye Model=1 kV=" 
                                + str(iact.node.kVbase_phg) + " kW=" + str(Pvec[0]*iact.node.kVAbase)
                                + " kvar=" + str(Qvec[0]*iact.node.kVAbase) + " Vminpu=0.8 Vmaxpu=1.2")
            if iact.phasevec[1,0] == 1:
                dss.run_command("New Load." + iact.name +"b Bus1=" + iact.node.name + ".2 Phases=1 Conn=Wye Model=1 kV=" 
                                + str(iact.node.kVbase_phg) + " kW=" + str(Pvec[1]*iact.node.kVAbase)
                                + " kvar=" + str(Qvec[1]*iact.node.kVAbase) + " Vminpu=0.8 Vmaxpu=1.2")
            if iact.phasevec[2,0] == 1:
                dss.run_command("New Load." + iact.name +"c Bus1=" + iact.node.name + ".3 Phases=1 Conn=Wye Model=1 kV=" 
                                + str(iact.node.kVbase_phg) + " kW=" + str(Pvec[2]*iact.node.kVAbase)
                                + " kvar=" + str(Qvec[2]*iact.node.kVAbase) + " Vminpu=0.8 Vmaxpu=1.2")
    return


# In[38]:


## FUNCTIONS FOR SOLVING NONLINEAR POWER FLOW
## Note that these store their solutions in the feeder object by updating Vmag_NL and Vang_NL at each bus


# In[39]:


def DSS_snapshot(feeder,timestep,subkVbase_phg,subkVAbase):
# Solves the nonlinear power flow for a single timestep 
    dss.run_command('clear')
    for key,inode in feeder.busdict.items():
        if inode.type == 'SLACK' or inode.type == 'Slack' or inode.type == 'slack':
            subbusname = inode.name

    # Sets up the new circuit, assumes 3ph
    dss.run_command("new circuit.currentckt basekv=" + str(subkVbase_phg*np.sqrt(3)) + " pu=1.0000 phases=3 bus1=" 
                    + subbusname + " Angle=0 MVAsc3=200000 MVASC1=200000")
    #dss.run_command("new circuit.currentckt basekv=" + str(subkVbase_phg*np.sqrt(3)) + " pu=1.0000 phases=3 bus1=" 
    #                + subbusname + " Angle=0 MVAsc3=232.4/3 MVASC1=232.4/3")
    DSS_loads(feeder,timestep)
    DSS_lines(feeder,timestep)
    DSS_caps(feeder,timestep)
    DSS_switches(feeder,timestep)
    DSS_trans(feeder,timestep)
    DSS_actuators(feeder,timestep)
    
    Vbases = str(subkVbase_phg) + "," + str(subkVbase_phg*np.sqrt(3))
    
    for key,inode in feeder.busdict.items():
        if inode.kVbase_phg != subkVbase_phg:
            Vbases = Vbases + "," + str(inode.kVbase_phg) + "," + str(inode.kVbase_phg*np.sqrt(3))
    
    dss.run_command("Set Voltagebases=[" + Vbases + "]")
    dss.run_command("set tolerance=0.0000000001")
    dss.run_command("calcv")
    dss.run_command("Solve")

def DSS_alltimesteps(feeder,alarm):
# Solves the nonlinear power flow for a time series
# 'Alarm' is a boolean that can be turned on or off. It prints a warning if any voltages are outside of bounds.
    subkVbase_phg = feeder.subkVbase_phg
    subkVAbase = feeder.subkVAbase
    loadpath = feeder.loadpath
    
    MVArating_check_list = []    
    for ts in range(0,feeder.timesteps):
        DSS_snapshot(feeder,ts,subkVbase_phg,subkVAbase)
        
        for key,bus in feeder.busdict.items():
            # Reset NL bus voltages
            bus.Vmag_NL[:,ts:ts+1] = np.zeros((3,1))
            bus.Vang_NL[:,ts:ts+1] = np.zeros((3,1))
            
            # Pull bus voltages out of the DSS solution
            name = bus.name
            dummyvar = dss.Circuit.SetActiveBus(bus.name)
            busVs = dss.Bus.VMagAngle()
            
            counter = 0
            for ph in bus.phases:
                if ph == 'a':
                    bus.Vmag_NL[0,ts] = busVs[counter]
                    bus.Vang_NL[0,ts] = busVs[counter + 1]
                    counter = counter + 2
                if ph == 'b':
                    bus.Vmag_NL[1,ts] = busVs[counter]
                    bus.Vang_NL[1,ts] = busVs[counter + 1]
                    counter = counter + 2
                if ph == 'c':
                    bus.Vmag_NL[2,ts] = busVs[counter]
                    bus.Vang_NL[2,ts] = busVs[counter + 1]
                    counter = counter

            if alarm == 1:
                if (bus.Vmag_NL[0,ts]/(1000*bus.kVbase_phg)>1.05 or (bus.Vmag_NL[0,ts]/(1000*bus.kVbase_phg)<0.95 and bus.phasevec[0,0]==1)):
                    print('Voltage violation: Phase 1, timestep ' + str(ts) + ' ' + bus.name)
                if (bus.Vmag_NL[1,ts]/(1000*bus.kVbase_phg)>1.05 or (bus.Vmag_NL[1,ts]/(1000*bus.kVbase_phg)<0.95 and bus.phasevec[1,0]==1)):
                    print('Voltage violation: Phase 2, timestep ' + str(ts) + ' ' + bus.name)
                if (bus.Vmag_NL[2,ts]/(1000*bus.kVbase_phg)>1.05 or (bus.Vmag_NL[2,ts]/(1000*bus.kVbase_phg)<0.95 and bus.phasevec[2,0]==1)):
                    print('Voltage violation: Phase 3, timestep ' + str(ts) + ' ' + bus.name)

# AMPACITY VIOLATIONS START
        for key,iline in feeder.linedict.items():
            
            if iline.MVArating_3ph != complex(0):
                # Reset NL line currents
                Imag_NL = np.zeros((3,1))
                Iang_NL = np.zeros((3,1))
        
                # Pull line currents out of the DSS solution
                name = iline.name
                dummyvar = dss.Circuit.SetActiveElement('Line.' + iline.name)
                lineIs = dss.CktElement.CurrentsMagAng()
                
                # Calculate ampacity from MVA rating (only written for balanced 3ph networks)
                ampacity = iline.MVArating_3ph*1000/3 / iline.from_node.kVbase_phg
                        
                counter = 0
                assert(iline.from_phases == iline.to_phases)
                for ph in iline.from_phases:
                    if ph == 'a':
                        iline.Imag_NL[0] = lineIs[counter]
                        iline.Iang_NL[0] = lineIs[counter + 1]
                        counter = counter + 2
                    if ph == 'b':
                        iline.Imag_NL[1] = lineIs[counter]
                        iline.Iang_NL[1] = lineIs[counter + 1]
                        counter = counter + 2
                    if ph == 'c':
                        iline.Imag_NL[2] = lineIs[counter]
                        iline.Iang_NL[2] = lineIs[counter + 1]
                        counter = counter
        
                if alarm == 1:
                    violation_vec = np.zeros(3)
                    #(print(f'{iline.name}_Imag_NL = {iline.Imag_NL[0,ts]//1}'))
                    for i in range(3):
                        if (iline.Imag_NL[i,ts] > ampacity and iline.phasevec[i,0]==1):
                            violation_vec[i] = 1
                    if any(violation_vec > 0):
                        violation_idx = np.where(violation_vec>0)[0]
                        phasekey = ['1','2','3']
                        print(f'*** AMPACITY VIOLATION @ {iline.name[5:]}: Phase(s) {[phasekey[i] for i in violation_idx]}, ts: {ts} ***')
                        print(f'{iline.name} Imag: {np.round(iline.Imag_NL[:,ts],2)}, amp = {np.round(ampacity,2)}')
                MVArating_check_list.append(1)
            else:
                MVArating_check_list.append(0)

# AMPACITY VIOLATIONS END
                    
        for key,iline in feeder.linedict.items():
            # Reset NL line currents
            iline.Imag_NL[:,ts:ts+1] = np.zeros((3,1))
            iline.Iang_NL[:,ts:ts+1] = np.zeros((3,1))
            
            # Pull bus voltages out of the DSS solution
            name = iline.name
            dummyvar = dss.Circuit.SetActiveElement('Line.' + iline.name)
            lineIs = dss.CktElement.CurrentsMagAng()
            
            counter = 0
            assert(iline.from_phases == iline.to_phases)
            for ph in iline.from_phases:
                if ph == 'a':
                    iline.Imag_NL[0,ts] = lineIs[counter]
                    iline.Iang_NL[0,ts] = lineIs[counter + 1]
                    counter = counter + 2
                if ph == 'b':
                    iline.Imag_NL[1,ts] = lineIs[counter]
                    iline.Iang_NL[1,ts] = lineIs[counter + 1]
                    counter = counter + 2
                if ph == 'c':
                    iline.Imag_NL[2,ts] = lineIs[counter]
                    iline.Iang_NL[2,ts] = lineIs[counter + 1]
                    counter = counter
                    
        for key,iline in feeder.transdict.items():
            # Reset NL line currents
            iline.Imag_NL[:,ts:ts+1] = np.zeros((3,1))
            iline.Iang_NL[:,ts:ts+1] = np.zeros((3,1))
            
            # Pull bus voltages out of the DSS solution
            name = iline.name
            
            for ph in iline.w0_phases:
                if ph == 'a':
                    dummyvar = dss.Circuit.SetActiveElement('Transformer.' + iline.name + 'A')
                    lineIs = dss.CktElement.CurrentsMagAng()
                    iline.Imag_NL[0,ts] = lineIs[0]
                    iline.Iang_NL[0,ts] = lineIs[1]

                if ph == 'b':
                    dummyvar = dss.Circuit.SetActiveElement('Transformer.' + iline.name + 'B')
                    lineIs = dss.CktElement.CurrentsMagAng()
                    iline.Imag_NL[1,ts] = lineIs[0]
                    iline.Iang_NL[1,ts] = lineIs[1]

                if ph == 'c':
                    dummyvar = dss.Circuit.SetActiveElement('Transformer.' + iline.name + 'C')
                    lineIs = dss.CktElement.CurrentsMagAng()
                    iline.Imag_NL[2,ts] = lineIs[0]
                    iline.Iang_NL[2,ts] = lineIs[1]
                    
        for key,iline in feeder.switchdict.items():
            # Reset NL line currents
            iline.Imag_NL[:,ts:ts+1] = np.zeros((3,1))
            iline.Iang_NL[:,ts:ts+1] = np.zeros((3,1))
            
            # Pull bus voltages out of the DSS solution
            name = iline.name
           
            for ph in iline.phases_from:
                if ph == 'a':
                    dummyvar = dss.Circuit.SetActiveElement('Line.' + iline.name + 'A')
                    lineIs = dss.CktElement.CurrentsMagAng()
                    iline.Imag_NL[0,ts] = lineIs[0]
                    iline.Iang_NL[0,ts] = lineIs[1]
                    
                if ph == 'b':
                    dummyvar = dss.Circuit.SetActiveElement('Line.' + iline.name + 'B')
                    lineIs = dss.CktElement.CurrentsMagAng()
                    iline.Imag_NL[1,ts] = lineIs[0]
                    iline.Iang_NL[1,ts] = lineIs[1]
                    
                if ph == 'c':
                    dummyvar = dss.Circuit.SetActiveElement('Line.' + iline.name + 'C')
                    lineIs = dss.CktElement.CurrentsMagAng()
                    iline.Imag_NL[2,ts] = lineIs[0]
                    iline.Iang_NL[2,ts] = lineIs[1]


    if all(MVArating_check_list) == False:
        print('* no MVA ratings provided for ampacity violations *')
    return


# In[41]:


## FOR REPORTING POWER FLOW SOLUTIONS


# In[42]:


def export_Vtargets(feeder):
# Turns the Vmag_NL and Vang_NL values within a feeder's bus dictionary into a csv file for the L-PBC team
# Also creates a csv with line currents, in case they find it useful for testing at LBL
    current_directory = os.getcwd()
    load_directory = feeder.loadfolder
    final_directory = os.path.join(load_directory, 'Results')
    if not os.path.exists(final_directory):
        os.makedirs(final_directory)
    os.chdir(final_directory)

    Vtargdf = pd.DataFrame()
    Idatdf = pd.DataFrame()
    KVbasedf = pd.DataFrame()

    for key,ibus in feeder.busdict.items():
        phAname = ibus.name + '_a'
        phBname = ibus.name + '_b'
        phCname = ibus.name + '_c'

        phAmag = ibus.Vmag_NL[0,:]/(ibus.kVbase_phg*1000)
        phBmag = ibus.Vmag_NL[1,:]/(ibus.kVbase_phg*1000)
        phCmag = ibus.Vmag_NL[2,:]/(ibus.kVbase_phg*1000)

        phAang = ibus.Vang_NL[0,:]
        phBang = ibus.Vang_NL[1,:]
        phCang = ibus.Vang_NL[2,:]

        Vtargdf[phAname + '_mag'] = phAmag
        Vtargdf[phAname + '_ang'] = phAang
        Vtargdf[phBname + '_mag'] = phBmag
        Vtargdf[phBname + '_ang'] = phBang
        Vtargdf[phCname + '_mag'] = phCmag
        Vtargdf[phCname + '_ang'] = phCang
        
    for key,iline in feeder.linedict.items():
        phAname = iline.name + '_a'
        phBname = iline.name + '_b'
        phCname = iline.name + '_c'

        phAmag = iline.Imag_NL[0,:]
        phBmag = iline.Imag_NL[1,:]
        phCmag = iline.Imag_NL[2,:]

        phAang = iline.Iang_NL[0,:]
        phBang = iline.Iang_NL[1,:]
        phCang = iline.Iang_NL[2,:]

        Idatdf[phAname + '_mag'] = phAmag
        Idatdf[phAname + '_ang'] = phAang
        Idatdf[phBname + '_mag'] = phBmag
        Idatdf[phBname + '_ang'] = phBang
        Idatdf[phCname + '_mag'] = phCmag
        Idatdf[phCname + '_ang'] = phCang

    for key,iline in feeder.transdict.items():
        phAname = iline.name + '_a'
        phBname = iline.name + '_b'
        phCname = iline.name + '_c'

        phAmag = iline.Imag_NL[0,:]
        phBmag = iline.Imag_NL[1,:]
        phCmag = iline.Imag_NL[2,:]

        phAang = iline.Iang_NL[0,:]
        phBang = iline.Iang_NL[1,:]
        phCang = iline.Iang_NL[2,:]

        Idatdf[phAname + '_mag'] = phAmag
        Idatdf[phAname + '_ang'] = phAang
        Idatdf[phBname + '_mag'] = phBmag
        Idatdf[phBname + '_ang'] = phBang
        Idatdf[phCname + '_mag'] = phCmag
        Idatdf[phCname + '_ang'] = phCang

    for key,iline in feeder.switchdict.items():
        phAname = iline.name + '_a'
        phBname = iline.name + '_b'
        phCname = iline.name + '_c'

        phAmag = iline.Imag_NL[0,:]
        phBmag = iline.Imag_NL[1,:]
        phCmag = iline.Imag_NL[2,:]

        phAang = iline.Iang_NL[0,:]
        phBang = iline.Iang_NL[1,:]
        phCang = iline.Iang_NL[2,:]

        Idatdf[phAname + '_mag'] = phAmag
        Idatdf[phAname + '_ang'] = phAang
        Idatdf[phBname + '_mag'] = phBmag
        Idatdf[phBname + '_ang'] = phBang
        Idatdf[phCname + '_mag'] = phCmag
        Idatdf[phCname + '_ang'] = phCang
        
    #[jasper] create output of KVbase values    
    for key,ibus in feeder.busdict.items():
        phAname = ibus.name + '_a'
        phBname = ibus.name + '_b'
        phCname = ibus.name + '_c'
        
        #phA_KVbase = ibus.Vmag_NL[0,:]/ibus.Vmag_NL[0,:]*ibus.kVbase_phg
        #phB_KVbase = ibus.Vmag_NL[1,:]/ibus.Vmag_NL[1,:]*ibus.kVbase_phg
        #phC_KVbase = ibus.Vmag_NL[2,:]/ibus.Vmag_NL[2,:]*ibus.kVbase_phg
        
        phA_KVbase = ibus.kVbase_phg
        phB_KVbase = ibus.kVbase_phg
        phC_KVbase = ibus.kVbase_phg
        
        
        KVbasedf[phAname + '_KVbase'] = [phA_KVbase]
        KVbasedf[phBname + '_KVbase'] = [phB_KVbase]
        KVbasedf[phCname + '_KVbase'] = [phC_KVbase]
        #[end]
        

    Vtargdf.to_csv('voltage_targets.csv')
    Idatdf.to_csv('current_data.csv')
    KVbasedf.to_csv('KVbase_values.csv')

    os.chdir(current_directory)
    return

#[jasper] create vectors to be sent to LPBC for HIL
# Create a single package to be read by specific LPBC's

def get_targets(feeder):
    
    print(get_targets)
    
    for key,ibus in feeder.busdict.items():
        if ibus.name == 'bus_671':
            #print(ibus.kVbase_phg)
            #print('NL671',ibus.Vmag_NL[0,0]/(ibus.kVbase_phg*1000))
            pass

    act_keys = []
    tstep_cur = 0 #need way to call most recent timestep?
    for key, inode in feeder.busdict.items():
        for iact in inode.actuators:
            act_keys.append(key)
#need way to set variable names automatically
#or other way to tag target vectors so that LPBC's can pull accordingly
# use ID #'s, and create map of ID's to nodes
    Vtarg_dict = {}
    for key,ibus in feeder.busdict.items():
        if ibus.type == 'SLACK' or ibus.type == 'Slack' or ibus.type == 'slack':
            phAmag_ref = ibus.Vmag_NL[0,tstep_cur]/(ibus.kVbase_phg*1000)
            phBmag_ref = ibus.Vmag_NL[1,tstep_cur]/(ibus.kVbase_phg*1000)
            phCmag_ref = ibus.Vmag_NL[2,tstep_cur]/(ibus.kVbase_phg*1000)
            
            phAang_ref = ibus.Vang_NL[0,tstep_cur]
            phBang_ref = ibus.Vang_NL[1,tstep_cur]
            phCang_ref = ibus.Vang_NL[2,tstep_cur]
    for key,ibus in feeder.busdict.items():
        if key in act_keys:
            phAmag = ibus.Vmag_NL[0,tstep_cur]/(ibus.kVbase_phg*1000)
            phBmag = ibus.Vmag_NL[1,tstep_cur]/(ibus.kVbase_phg*1000)
            phCmag = ibus.Vmag_NL[2,tstep_cur]/(ibus.kVbase_phg*1000)
              
            phAang = ibus.Vang_NL[0,tstep_cur]
            phBang = ibus.Vang_NL[1,tstep_cur]
            phCang = ibus.Vang_NL[2,tstep_cur]
            
            if np.abs(ibus.Vmag_NL[0,tstep_cur]) > 0:
                phA_kVbase = ibus.Vmag_NL[0,tstep_cur]/ibus.Vmag_NL[0,tstep_cur]*ibus.kVbase_phg
            else:
                phA_kVbase = np.nan
            if np.abs(ibus.Vmag_NL[1,tstep_cur]) > 0:
                phB_kVbase = ibus.Vmag_NL[1,tstep_cur]/ibus.Vmag_NL[1,tstep_cur]*ibus.kVbase_phg
            else:
                phA_kVbase = np.nan
            if np.abs(ibus.Vmag_NL[2,tstep_cur]) > 0:
                phC_kVbase = ibus.Vmag_NL[2,tstep_cur]/ibus.Vmag_NL[2,tstep_cur]*ibus.kVbase_phg
            else:
                phA_kVbase = np.nan
            
            Vtarg_dict[key] = {}
            Vtarg_dict[key]['Vmag'] = [phAmag-phAmag_ref,phBmag-phBmag_ref,phCmag-phCmag_ref]
            #Vtarg_dict[key]['Vmag'] = [phAmag-refphasor[0,0],phBmag-[phAmag-refphasor[1,0],phCmag-phCmag_ref]
            Vtarg_dict[key]['Vang'] = [phAang-phAang_ref,phBang-phBang_ref,phCang-phCang_ref]
            Vtarg_dict[key]['KVbase'] = [phA_kVbase,phB_kVbase,phC_kVbase]
            Vtarg_dict[key]['KVAbase'] = [feeder.subkVAbase/3,feeder.subkVAbase/3,feeder.subkVAbase/3] #assumes 3ph sub
            
    return Vtarg_dict, act_keys

def get_gen(feeder):
    gendict = {}
    for key, iact in feeder.actdict.items():
        Pgen = iact.Pgen[:,:].value*iact.node.kVAbase
        Qgen = iact.Qgen[:,:].value*iact.node.kVAbase
        gendict[key] = {
                'P': Pgen,
                'Q': Qgen
                    }
    return gendict

def get_shunt(feeder):
    shuntgendict = {}
    for key, icap in feeder.shuntdict.items():
        if icap.id[0:8] == 'sync_cap':
            Qgen = icap.Qgen[:,:].value*icap.node.kVAbase
            shuntgendict[key] = {
                    'Q': Qgen
                        }
    return shuntgendict

def get_flow(feeder):
    flowdict = {}
    conndict = {}
    for key, iconn in feeder.linedict.items():
        conndict[key] = iconn
    for key, iconn in feeder.transdict.items():
        conndict[key] = iconn
    for key, iconn in feeder.switchdict.items():
        conndict[key] = iconn
        
    for key, iconn in conndict.items():
        if isinstance(iconn,line):
            Pflow = iconn.P_linopt[:,:].value*iconn.kVAbase
            Qflow = iconn.Q_linopt[:,:].value*iconn.kVAbase
        if isinstance(iconn,transformer):
            Pflow = iconn.P_linopt[:,:].value*iconn.w0_kVAbase
            Qflow = iconn.Q_linopt[:,:].value*iconn.w0_kVAbase
        flowdict[key] = {
                'P': Pflow,
                'Q': Qflow
                    }
    return flowdict

# this code is essentially the same as above but instead of exporting csv files it converts to numpy vectors
# This iteration of code is to produce a single vector to be ingested by every LPBC - Not using currently
    '''
def get_targets(feeder):
    
# create dataframes for targets and associated values
    Vmagdf = pd.DataFrame()
    Vangdf = pd.DataFrame()
    Idatdf = pd.DataFrame()
    KVbasedf = pd.DataFrame()
#Vmagdf & Vangdf
    for key,ibus in feeder.busdict.items():
        phAname = ibus.name + '_a'
        phBname = ibus.name + '_b'
        phCname = ibus.name + '_c'

        phAmag = ibus.Vmag_NL[0,:]/(ibus.kVbase_phg*1000)
        phBmag = ibus.Vmag_NL[1,:]/(ibus.kVbase_phg*1000)
        phCmag = ibus.Vmag_NL[2,:]/(ibus.kVbase_phg*1000)

        phAang = ibus.Vang_NL[0,:]
        phBang = ibus.Vang_NL[1,:]
        phCang = ibus.Vang_NL[2,:]

        Vmagdf[phAname + '_mag'] = phAmag
        Vangdf[phAname + '_ang'] = phAang
        Vmagdf[phBname + '_mag'] = phBmag
        Vangdf[phBname + '_ang'] = phBang
        Vmagdf[phCname + '_mag'] = phCmag
        Vangdf[phCname + '_ang'] = phCang

#Idatdf        
    for key,iline in feeder.linedict.items():
        phAname = iline.name + '_a'
        phBname = iline.name + '_b'
        phCname = iline.name + '_c'

        phAmag = iline.Imag_NL[0,:]
        phBmag = iline.Imag_NL[1,:]
        phCmag = iline.Imag_NL[2,:]

        phAang = iline.Iang_NL[0,:]
        phBang = iline.Iang_NL[1,:]
        phCang = iline.Iang_NL[2,:]

        Idatdf[phAname + '_mag'] = phAmag
        Idatdf[phAname + '_ang'] = phAang
        Idatdf[phBname + '_mag'] = phBmag
        Idatdf[phBname + '_ang'] = phBang
        Idatdf[phCname + '_mag'] = phCmag
        Idatdf[phCname + '_ang'] = phCang

    for key,iline in feeder.transdict.items():
        phAname = iline.name + '_a'
        phBname = iline.name + '_b'
        phCname = iline.name + '_c'

        phAmag = iline.Imag_NL[0,:]
        phBmag = iline.Imag_NL[1,:]
        phCmag = iline.Imag_NL[2,:]

        phAang = iline.Iang_NL[0,:]
        phBang = iline.Iang_NL[1,:]
        phCang = iline.Iang_NL[2,:]

        Idatdf[phAname + '_mag'] = phAmag
        Idatdf[phAname + '_ang'] = phAang
        Idatdf[phBname + '_mag'] = phBmag
        Idatdf[phBname + '_ang'] = phBang
        Idatdf[phCname + '_mag'] = phCmag
        Idatdf[phCname + '_ang'] = phCang

    for key,iline in feeder.switchdict.items():
        phAname = iline.name + '_a'
        phBname = iline.name + '_b'
        phCname = iline.name + '_c'

        phAmag = iline.Imag_NL[0,:]
        phBmag = iline.Imag_NL[1,:]
        phCmag = iline.Imag_NL[2,:]

        phAang = iline.Iang_NL[0,:]
        phBang = iline.Iang_NL[1,:]
        phCang = iline.Iang_NL[2,:]

        Idatdf[phAname + '_mag'] = phAmag
        Idatdf[phAname + '_ang'] = phAang
        Idatdf[phBname + '_mag'] = phBmag
        Idatdf[phBname + '_ang'] = phBang
        Idatdf[phCname + '_mag'] = phCmag
        Idatdf[phCname + '_ang'] = phCang
#KVbasedf          
    for key,ibus in feeder.busdict.items():
        phAname = ibus.name + '_a'
        phBname = ibus.name + '_b'
        phCname = ibus.name + '_c'
        
        phA_KVbase = ibus.Vmag_NL[0,:]/ibus.Vmag_NL[0,:]*ibus.kVbase_phg
        phB_KVbase = ibus.Vmag_NL[1,:]/ibus.Vmag_NL[1,:]*ibus.kVbase_phg
        phC_KVbase = ibus.Vmag_NL[2,:]/ibus.Vmag_NL[2,:]*ibus.kVbase_phg
        
        KVbasedf[phAname + '_KVbase'] = phA_KVbase
        KVbasedf[phBname + '_KVbase'] = phB_KVbase
        KVbasedf[phCname + '_KVbase'] = phC_KVbase
        
## create vectors from dataframes
        
# get array of nodes on feeder (this order will correspond to the order of the vectors)
    keys = []
    for key,ibus in feeder.busdict.items():
        keys.append(key)
    nodes_arr = np.array(keys)
#Vmag        
    Vmag_targ = np.ones((len(nodes_arr),3))
    for i in range(len(nodes_arr)):
        y = i*3
        #this is assuming a single timestep is used for each iteration
        Vmag_targ[i,:] = Vmagdf[Vmagdf.columns[y:y+3]].values[0]
#Vang
    Vang_targ = np.ones((len(nodes_arr),3))
    for i in range(len(nodes_arr)):
        y = i*3
        #this is assuming a single timestep is used for each iteration
        Vang_targ[i,:] = Vangdf[Vangdf.columns[y:y+3]].values[0]
#KVbase
    KVbase = np.ones((len(nodes_arr),3))
    for i in range(len(nodes_arr)):
        y = i*3
        #this is assuming a single timestep is used for each iteration
        KVbase[i,:] = KVbasedf[KVbasedf.columns[y:y+3]].values[0]
    return(nodes_arr,Vmag_targ,Vang_targ,KVbase)
    '''
    
        
 

    
    
    
    
    
    
    
    
    