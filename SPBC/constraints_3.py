# THIS SCRIPT DEFINES THE CONSTRAINTS OF MICHAEL SANKUR'S LUPFM MODEL
#SPBC-HIL files moved 7/15/19

# In[2]:

from setup_3 import *
import datetime

# In[3]:


# HELPER FUNCTIONS


# In[4]:

def cvx_setzeros(cvxvar, phasevec, timesteps):
# Given a cvx variable, set it equal to zero on the phases where phasevec==0 for all timesteps
    conslist = list()
    for ts in range(0, timesteps):
        for idx in range(0, 3):
            if phasevec[idx,0]==0:
                conslist.append(cvxvar[idx,ts]==0)
    return conslist

def loadRX2Y(Rvec,Xvec):
# Convert a 3x1 vector of Rs and a 3x1 vector of Xs to a 3x3 diagonal Y matrix for multiplying Vmag^2
    Ymat = np.zeros((3,3), dtype=np.complex_)
    for idx in range(0,3):
        if Rvec[idx,0]!=0 or Xvec[idx,0]!=0:
            Ymat[idx,idx]=1./(Rvec[idx,0]-1j*Xvec[idx,0]) # Remember that we're using the conjugate here
    return Ymat

def cvx_setrealdemandpu(currentnode,timestep):
# Create an expression for the real power demand at each load (p.u.)
    demand = 0.
    for iload in currentnode.loads:
        cP = iload.constP
        cZ = iload.constZ
        cI = iload.constI
        
        Sbase = currentnode.kVAbase
        Vbase = currentnode.kVbase_phg
        Zbase = currentnode.Zbase
        
        currPpu = iload.Psched[:, timestep:timestep+1]/Sbase
        currRpu = iload.Rsched[:, timestep:timestep+1]/Zbase
        currXpu = iload.Xsched[:, timestep:timestep+1]/Zbase
        
        demand = demand + currPpu*(cP + cI)
        demand = demand + np.real(cZ*loadRX2Y(currRpu,currXpu))*currentnode.Vmagsq_linopt[:, timestep:timestep+1]
    
    return demand

def cvx_setreactivedemandpu(currentnode,timestep):
# Create an expression for the reactive power demand at each load (p.u.)
    demand = 0.
    for iload in currentnode.loads:
        cP = iload.constP
        cZ = iload.constZ
        cI = iload.constI
        
        Sbase = currentnode.kVAbase
        Vbase = currentnode.kVbase_phg
        Zbase = currentnode.Zbase
        
        currQpu = iload.Qsched[:, timestep:timestep+1]/Sbase
        currRpu = iload.Rsched[:, timestep:timestep+1]/Zbase
        currXpu = iload.Xsched[:, timestep:timestep+1]/Zbase
        
        demand = demand + currQpu*(cP + cI)
        demand = demand + np.imag(cZ*loadRX2Y(currRpu,currXpu))*currentnode.Vmagsq_linopt[:, timestep:timestep+1]
    
    for icap in currentnode.cap:
        Sbase = currentnode.kVAbase
        Vbase = currentnode.kVbase_phg
        Zbase = currentnode.Zbase
        
        # Caps are read in with a negative sign, so this works out right
        demand = demand + icap.Qvec/Sbase
    
    return demand

def magang2complex(magvec,angvec):
# Create a complex number from a column of magnitudes and a column of angles
    assert(len(magvec) == len(angvec)),"Vector length problems"
    outvec = np.zeros((len(magvec), 1), dtype=np.complex_)
    for idx in range(0,len(magvec)):
        outvec[idx,0] = magvec[idx,0]*np.cos(np.deg2rad(angvec[idx,0]))+1j*magvec[idx,0]*np.sin(np.deg2rad(angvec[idx,0]))
    return outvec

def cvx_buildgamma(currentline,timestep):
# A helper function necessary for the LUPFM (Having Michael's dissertation open as a reference will be useful in understanding these constraints)
# Note that this is referenced to receiving end
    if isinstance(currentline,transformer):
        Vtomag = currentline.w1_node.Vmag_NL[:, timestep:timestep+1]
        Vtoang = currentline.w1_node.Vang_NL[:, timestep:timestep+1]
    else:
        Vtomag = currentline.to_node.Vmag_NL[:, timestep:timestep+1]
        Vtoang = currentline.to_node.Vang_NL[:, timestep:timestep+1]

    Vto = magang2complex(Vtomag,Vtoang)
    
    gammaout = np.zeros((len(Vto), len(Vto)), dtype=np.complex_)
    
    for idx1 in range(0,len(Vto)):
        if currentline.phasevec[idx1,0] != 0:
            for idx2 in range(0,len(Vto)):
                if currentline.phasevec[idx2,0] != 0:
                    gammaout[idx1,idx2] = Vto[idx1]/Vto[idx2]
                    
    
    return gammaout

def cvx_buildM(currentline,timestep):
# A helper function necessary for the LUPFM
    gamma = cvx_buildgamma(currentline,timestep)
    return np.real(np.multiply(gamma,np.conj(currentline.Zpu)))

def cvx_buildN(currentline,timestep):
# A helper function necessary for the LUPFM
    gamma = cvx_buildgamma(currentline,timestep)
    return np.imag(np.multiply(gamma,np.conj(currentline.Zpu)))


# In[5]:


# CONSTRAINT DEFINITION FUNCTIONS


# In[6]:


def cons_slack(feeder):
# This will set slack bus magnitude to 1 and angles to 0, 4pi/3, 2pi/3.
# A more sophisticated later version might set the voltages at multiple buses based on Vsrc objects.
    #[HIL] - refphasor - set slack = to refphasor
    conslist = list()
    for key, inode in feeder.busdict.items():
        if inode.type == 'SLACK' or inode.type == 'Slack' or inode.type == 'slack':
            for idx in range(feeder.timesteps):
                conslist.append(inode.Vmagsq_linopt[:,idx:idx+1] == feeder.refphasor[:,0:1])
                conslist.append(inode.Vang_linopt[:,idx:idx+1] == feeder.refphasor[:,1:2])        
                '''
                conslist.append(inode.Vmagsq_linopt[:,idx:idx+1] == np.ones([3,1]))
                conslist.append(inode.Vang_linopt[:,idx:idx+1] == np.array([[0],[4*np.pi/3],[2*np.pi/3]]))
                '''
    return conslist


# In[7]:


def cons_missingnode(feeder):
# This will set any nonexistent phase voltages at a node to zero, along with the powers on lines attached to those phases
    conslist = list()
    for key, inode in feeder.busdict.items():
        conslist = conslist + cvx_setzeros(inode.Vmagsq_linopt, inode.phasevec, feeder.timesteps)
        conslist = conslist + cvx_setzeros(inode.Vang_linopt, inode.phasevec, feeder.timesteps)
        for iline in inode.edges_in:
            conslist = conslist + cvx_setzeros(iline.P_linopt, inode.phasevec, feeder.timesteps)
            conslist = conslist + cvx_setzeros(iline.Q_linopt, inode.phasevec, feeder.timesteps)
        for iline in inode.edges_out:
            conslist = conslist + cvx_setzeros(iline.P_linopt, inode.phasevec, feeder.timesteps)
            conslist = conslist + cvx_setzeros(iline.Q_linopt, inode.phasevec, feeder.timesteps)
    return conslist


# In[8]:


def cons_missingline(feeder):
# This will set the powers on any nonexistent lines to 0 (covers any missed by cons_missingnode)
    conslist = list()
    for key, iline in feeder.linedict.items():
        assert (iline.from_phases == iline.to_phases), "Phases across lines don't match"
        conslist = conslist + cvx_setzeros(iline.P_linopt,iline.phasevec, feeder.timesteps)
        conslist = conslist + cvx_setzeros(iline.Q_linopt,iline.phasevec, feeder.timesteps)
    return conslist


# In[9]:


def cons_Vineq(feeder):
# Sets upper and lower bounds on nodal voltages. The allowable band may need to be reduced to get good nonlinear solutions!
    conslist = list()
    for key, inode in feeder.busdict.items():
        for ts in range(0,feeder.timesteps):
            V = inode.Vmagsq_linopt[:,ts:ts+1]
            for idx in range(0,3):
                if inode.phasevec[idx,0] != 0:
                    conslist.append(V[idx,0] <= 1.03*1.03)
                    conslist.append(V[idx,0] >= 0.9*0.9)
    return conslist


# In[10]:


def cons_realpwrbalance(feeder):
# An expression for the p.u. real power balance at each node
    conslist = list()
    for key, inode in feeder.busdict.items():
        if inode.type != 'SLACK' and inode.type != 'Slack' and inode.type != 'slack':
            for ts in range(0,feeder.timesteps):
                power_in = np.zeros((3,1), dtype=np.complex_)
                for iedgein in inode.edges_in:
                    power_in = power_in + iedgein.P_linopt[:,ts:ts+1]

                power_out = np.zeros((3,1), dtype=np.complex_)
                for iedgeout in inode.edges_out:
                    power_out = power_out + iedgeout.P_linopt[:,ts:ts+1]

                actuation = np.zeros((3,1), dtype=np.complex_)
                for iact in inode.actuators:
                    actuation = (actuation + iact.Pgen[:,ts:ts+1])

                conslist.append((power_in-power_out) == (cvx_setrealdemandpu(inode,ts)-actuation))
    return conslist

def cons_reactivepwrbalance(feeder):
# An expression for the p.u. reactive power balance at each node
    conslist = list()
    for key, inode in feeder.busdict.items():
        if inode.type != 'SLACK' and inode.type != 'Slack' and inode.type != 'slack':
            for ts in range(0,feeder.timesteps):
                power_in = np.zeros((3,1), dtype=np.complex_) 
                for iedgein in inode.edges_in:
                    power_in = power_in + iedgein.Q_linopt[:,ts:ts+1]

                power_out = np.zeros((3,1), dtype=np.complex_)
                for iedgeout in inode.edges_out:
                    power_out = power_out + iedgeout.Q_linopt[:,ts:ts+1]

                actuation = np.zeros((3,1), dtype=np.complex_)
                for iact in inode.actuators:
                    actuation = actuation + iact.Qgen[:,ts:ts+1]

                conslist.append(power_in-power_out == cvx_setreactivedemandpu(inode,ts)-actuation)
    return conslist

# In[11a]:
    
### Calculate weight for PV for real time simulation ### 

# calculate solar radiation (adapted from Masters)
#inputs of lat, lon, maridian [degrees] & fraction of PV [fraction i.e. 0.5=50%].
def solweight_realtime(lat,lon,maridian): #[HIL]
    deg_rad = np.pi/180.
    rad_deg = 180./np.pi

    lat = lat*deg_rad
    
    date = datetime.datetime.now()
    month = date.month
    day = date.day
    hour = date.hour
    minute = date.minute
    DOY = datetime.datetime.now().timetuple().tm_yday
    
    dsol = 23.45*np.sin(360/365*(DOY-81)*deg_rad)*deg_rad  #solar declination
    Hsol = 15*(12-(hour+(minute+(lon-maridian)*4)/60))*deg_rad  #hour angle

    alt = np.arcsin(np.cos(lat)*np.cos(dsol)*np.cos(Hsol)+np.sin(lat)*np.sin(dsol))  #solar altitude
    azi = np.arcsin(np.cos(dsol)*np.sin(Hsol)/np.cos(alt))  #solar azimuth

    Asol = 1160 + 75*np.sin(360/365*(DOY-275)*deg_rad)  #[W/m^2]  #extraterrestial flux
    ksol = 0.174 + 0.035*np.sin(360/365*(DOY-100)*deg_rad)  #optical depth
    msol = np.sqrt((708*np.sin(alt))**2 + 1417) - 708*np.sin(alt)  #air mass ratio

    Ib = Asol*np.e**(-ksol*msol)  #[W/m^2]  #beam radiation
    solweight = Ib/1000
    
    return solweight

#currently tilt is not included, assumed that panel always faces sun directly as to avoid underestimating
#seems implausible to know the tilt of every array in the network
    
# In[11b]:


# def cons_actuators(feeder,acttoggle):
#     # Imports constraints on the actuators from the appropriate excel file
#     # acttoggle can turn actuators off altogether
#     # This version creates the constraints as a full QCQP
#     conslist = list()
#     for key, inode in feeder.busdict.items():
#         for iact in inode.actuators:
#             for ts in range(0,feeder.timesteps):
#                 if acttoggle == True:
#                     conslist.append(cp.abs(iact.Pgen[:,ts:ts+1]) <= iact.Psched[:,ts:ts+1]/inode.kVAbase)
#                     for idx in range(0,3):
#                         conslist.append(cp.square(iact.Pgen[idx,ts])+cp.square(iact.Qgen[idx,ts]) \
#                                         <= cp.square(iact.Ssched[idx,ts]/inode.kVAbase))
#                 else:
#                     for idx in range(0,3):
#                         conslist.append(iact.Pgen[idx,ts] == 0)
#                         conslist.append(iact.Qgen[idx,ts] == 0)
#     return conslist       

def cons_actuators(feeder,acttoggle):
    # Imports constraints on the actuators from the appropriate excel file
    # acttoggle can turn actuators off altogether
    # This version creates box constraints and can be used if the circular constraints defined above cause problems
    conslist = list()
    #for key, inode in feeder.busdict.items():
    for key, inode in feeder.busdict.items():
        
        # Creates a feedback for saturated actuators.
        # This is a very basic implementation, should be improved in later versions
        
        #[HIL] - ICDI
        Psatmul = [1,1,1]
        Qsatmul = [1,1,1]
        for phidx, ph in enumerate (['a','b','c']):
            if any(key in x for x in feeder.Psat_nodes) & any(ph in x for x in feeder.Psat_nodes) == True:
                Psatmul[phidx] = 0.8
            if any(key in x for x in feeder.Qsat_nodes) & any(ph in x for x in feeder.Qsat_nodes) == True:
                Qsatmul[phidx] = 0.8
        for iact in inode.actuators:
            #scale PV actuation down based on TOD irradiance (really only uses beam) [HIL]
            if feeder.PVforecast[str(key)]['on_off'] == True:
                solweight = solweight_realtime(feeder.PVforecast[str(key)]['lat'],feeder.PVforecast[str(key)]['lon'],
                                                feeder.PVforecast[str(key)]['maridian'])
                PVfrac = feeder.PVforecast[str(key)]['PVfrac']
            else:
                solweight = 0
                PVfrac = 0
            for ts in range(0,feeder.timesteps):
                if acttoggle == True:
                    for idx in range(0,3):
                        #adjust max Pgen to be scaled by insolation [HIL]
                        Pmax = iact.Psched[idx,ts:ts+1]*(1-PVfrac*(1-solweight))
                        conslist.append(cp.abs(iact.Pgen[idx,ts:ts+1]) <= (Pmax*Psatmul[idx])/inode.kVAbase)  #[HIL] - ICDI
                        #conslist.append(cp.abs(iact.Pgen[idx,ts:ts+1]) <= (iact.Psched[idx,ts:ts+1]*Psatmul)/inode.kVAbase)  #[HIL] - ICDI
                        #[HIL] - edit Ssched - Qgen cons
                        conslist.append(cp.abs(iact.Qgen[idx,ts:ts+1]) <= ((iact.Ssched[idx,ts:ts+1]-cp.abs(iact.Pgen[idx,ts:ts+1])*inode.kVAbase)*Qsatmul[idx])/inode.kVAbase) #new
                        #conslist.append(cp.abs(iact.Qgen[idx,ts:ts+1]) <= ((iact.Ssched[idx,ts:ts+1]-iact.Psched[idx,ts:ts+1])*Qsatmul)/inode.kVAbase) #old
                    
                    
                else:
                    for idx in range(0,3):
                        conslist.append(iact.Pgen[idx,ts] == 0)
                        conslist.append(iact.Qgen[idx,ts] == 0)
                        
    return conslist   


# In[12]:


def cons_Mageq(feeder):
# Enforces the relationships between voltage magnitudes and power flows specified by the LUPFM
    conslist = list()
    
    # Lines
    for key, iline in feeder.linedict.items():
        for ts in range(0,feeder.timesteps):
            M = cvx_buildM(iline,ts)
            N = cvx_buildN(iline,ts)
            Vsqsend = iline.from_node.Vmagsq_linopt[:,ts:ts+1]
            Vsqrec = iline.to_node.Vmagsq_linopt[:,ts:ts+1]
            P = iline.P_linopt[:,ts:ts+1]
            Q = iline.Q_linopt[:,ts:ts+1]
            
            for idx in range(0,3):
                if iline.phasevec[idx,0] !=0:
                    conslist.append(Vsqsend[idx,0] == Vsqrec[idx,0] + 2*M[idx,:]*P - 2*N[idx,:]*Q)
                    
    # Switches
    for key, iline in feeder.switchdict.items():
        for ts in range(0,feeder.timesteps):
            M = cvx_buildM(iline,ts)
            N = cvx_buildN(iline,ts)
            Vsqsend = iline.from_node.Vmagsq_linopt[:,ts:ts+1]
            Vsqrec = iline.to_node.Vmagsq_linopt[:,ts:ts+1]
            P = iline.P_linopt[:,ts:ts+1]
            Q = iline.Q_linopt[:,ts:ts+1]
            
            for idx in range(0,3):
                if iline.phasevec[idx,0] !=0:
                    conslist.append(Vsqsend[idx,0] == Vsqrec[idx,0] + 2*M[idx,:]*P - 2*N[idx,:]*Q)
                    
    # Transformers
    for key, iline in feeder.transdict.items():
        for ts in range(0,feeder.timesteps):
            M = cvx_buildM(iline,ts)
            N = cvx_buildN(iline,ts)
            Vsqsend = iline.w0_node.Vmagsq_linopt[:,ts:ts+1]
            Vsqrec = iline.w1_node.Vmagsq_linopt[:,ts:ts+1]
            P = iline.P_linopt[:,ts:ts+1]
            Q = iline.Q_linopt[:,ts:ts+1]
            
            for idx in range(0,3):
                if iline.phasevec[idx,0] !=0:
                    conslist.append(Vsqsend[idx,0] == Vsqrec[idx,0] + 2*M[idx,:]*P - 2*N[idx,:]*Q)
    return conslist


# In[13]:


def cons_Angeq(feeder):
# Enforces the relationships between voltage magnitudes and power flows specified by the LUPFM
    conslist = list()
    
    # Lines
    for key, iline in feeder.linedict.items():
        for ts in range(0,feeder.timesteps):
            M = cvx_buildM(iline,ts)
            N = cvx_buildN(iline,ts)
            Vangsend = iline.from_node.Vang_linopt[:,ts:ts+1]
            Vangrec = iline.to_node.Vang_linopt[:,ts:ts+1]
            P = iline.P_linopt[:,ts:ts+1]
            Q = iline.Q_linopt[:,ts:ts+1]
            Vvec = np.multiply(iline.from_node.Vmag_NL,iline.to_node.Vmag_NL)
            for idx in range(0,3):
                if iline.phasevec[idx,0] !=0:
                    conslist.append(Vvec[idx,0]*(Vangsend[idx,0]-Vangrec[idx,0]) == -N[idx,:]*P - M[idx,:]*Q)
                    
    # Switches
    for key, iline in feeder.switchdict.items():
        for ts in range(0,feeder.timesteps):
            M = cvx_buildM(iline,ts)
            N = cvx_buildN(iline,ts)
            Vangsend = iline.from_node.Vang_linopt[:,ts:ts+1]
            Vangrec = iline.to_node.Vang_linopt[:,ts:ts+1]
            P = iline.P_linopt[:,ts:ts+1]
            Q = iline.Q_linopt[:,ts:ts+1]
            Vvec = np.multiply(iline.from_node.Vmag_NL,iline.to_node.Vmag_NL)
            for idx in range(0,3):
                if iline.phasevec[idx,0] !=0:
                    conslist.append(Vvec[idx,0]*(Vangsend[idx,0]-Vangrec[idx,0]) == -N[idx,:]*P - M[idx,:]*Q)
                    
    # Transformers
    for key, iline in feeder.transdict.items():
        for ts in range(0,feeder.timesteps):
            M = cvx_buildM(iline,ts)
            N = cvx_buildN(iline,ts)
            Vangsend = iline.w0_node.Vang_linopt[:,ts:ts+1]
            Vangrec = iline.w1_node.Vang_linopt[:,ts:ts+1]
            P = iline.P_linopt[:,ts:ts+1]
            Q = iline.Q_linopt[:,ts:ts+1]
            Vvec = np.multiply(iline.w0_node.Vmag_NL,iline.w1_node.Vmag_NL)
            for idx in range(0,3):
                if iline.phasevec[idx,0] !=0:
                    conslist.append(Vvec[idx,0]*(Vangsend[idx,0]-Vangrec[idx,0]) == -N[idx,:]*P - M[idx,:]*Q)
                    
    return conslist


# In[14]:


def cvx_set_constraints(feeder, acttoggle):
# Sets all constraints
# Acttoggle toggles actuators on or off. It also removes the inequality bounds on in the case where actuators are disabled.
    conslist = cons_slack(feeder)
    conslist = conslist + cons_missingnode(feeder) 
    conslist = conslist + cons_missingline(feeder)
    conslist = conslist + cons_realpwrbalance(feeder) 
    conslist = conslist + cons_reactivepwrbalance(feeder) 
    
    if acttoggle == True:
        conslist = conslist + cons_Vineq(feeder)
         
    conslist = conslist + cons_actuators(feeder, acttoggle)
    conslist = conslist + cons_Mageq(feeder)
    conslist = conslist + cons_Angeq(feeder)
    return conslist