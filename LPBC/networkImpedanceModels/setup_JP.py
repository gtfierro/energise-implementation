# In[22]:


## THIS SCRIPT STORES THE FUNCTIONS AND CLASSES NECESSARY TO CREATE A 'FEEDER' OBJECT FROM AN EPHASORSIM MODEL


# In[23]:

import numpy as np
import pandas as pd
#import cvxpy as cp
import string
import networkx as nx # As of 2018.5.17 you need networkx v1.9; later versions have lost some important functionality
import matplotlib.pyplot as plt
from copy import deepcopy
import pdb


# In[24]:


# Adds traceback to warning messages, which is sometimes useful
import traceback
import warnings
import sys

print_network_values = 0

def warn_with_traceback(message, category, filename, lineno, file=None, line=None):

    log = file if hasattr(file,'write') else sys.stderr
    traceback.print_stack(file=log)
    log.write(warnings.formatwarning(message, category, filename, lineno, line))

warnings.showwarning = warn_with_traceback


# In[25]:


## CLASS DEFINITIONS/HELPER FUNCTIONS


# In[26]:

#Keith: I find this feeder structure is a bit confusing bc it has all of the measurements in bus classes, which are a part of the parent "feeder" class
#If I redo it, I might change so theres a feeder class, a load class, and a results class, to keep things seperate
class feeder:
#each edge in feeder has a pu and nonpu Z and Y, except for transformers which just have pu (I think)
# The main class. Creating this from an ePHASORsim model and set of loads stores all the information about your network
# Buses, loads, lines, actuators, etc. are all stored as dictionaries within this object
    def __init__(self,modelpath,loadfolder,loadpath,actpath,timesteps,subkVbase_phg,subkVAbase):
        # import data
        self.modeldata = pd.ExcelFile(modelpath)
        self.loadfolder = loadfolder
        self.loadpath = loadpath
        self.actpath = actpath
        self.timesteps = timesteps
        self.subkVbase_phg = subkVbase_phg
        self.subkVAbase = subkVAbase
        # groom data (note that the functions below are not separable...
        # ...e.g. buses are not completely defined until all these functions have been run)
        self.busdict = busbuilder(self.modeldata, subkVbase_phg, subkVAbase, timesteps)
        self.Vsrcdict = Vsrcbuilder(self.modeldata, self.busdict)
        self.shuntdict = shuntbuilder(self.modeldata, self.busdict,timesteps)
        self.loaddict = loadbuilderPQ(self.modeldata, self.busdict, loadpath, timesteps)
        self.actdict = actbuilder(self.modeldata, self.busdict, loadpath, timesteps)
        self.linedict = linebuilder(self.modeldata, self.busdict, timesteps)
        self.transdict = transbuilder(self.modeldata, self.busdict, self.subkVAbase, timesteps)
        self.switchdict = switchbuilder(self.modeldata, self.busdict, timesteps)
        self.network = network_mapper(self.modeldata,self.busdict,self.linedict,self.transdict,self.switchdict)
        set_per_unit(self.linedict) #line PU values are set here for Zpu, Ypu (and Zbase)
        # maybe want to model shunt caps as their impedance, would need to put it in pu (but DSS takes in shunt caps in terms of power as well)

class bus:
    def __init__(self,desig,subkVbase_phg,subkVAbase,timesteps):
        self.name = 'bus_' + str(desig)
        self.buskey = desig
        self.phases = list()
        self.phasevec = np.zeros((3,1))
        self.type = ''
        self.kVbase_phg = subkVbase_phg
        self.kVAbase = subkVAbase
        self.Zbase = subkVbase_phg*subkVbase_phg*1000/subkVAbase
        self.loads = list()
        self.cap = list()
        self.actuators = list()
        self.edges_out = list()
        self.edges_in = list()
        self.Vsrcs = list()

        #self.Vmagsq_linopt = cp.Variable((3,timesteps), name=self.name + '_Vmagsq')
        #self.Vang_linopt = cp.Variable((3,timesteps), self.name + '_Vang')

        self.Vmag_NL = np.ones((3,timesteps))
        self.Vang_NL = np.array([[0.],[240.],[120.]])
        # should change these so angle ares all base 0 and in radians
        self.Imag_NL = np.zeros((3,timesteps))
        self.Iang_NL = np.array([[0.],[240.],[120.]])
        for idx in range(timesteps-1):
            self.Vang_NL = np.concatenate((self.Vang_NL,np.array([[0.],[240.],[120.]])), axis=1)
            self.Iang_NL = np.concatenate((self.Iang_NL,np.array([[0.],[240.],[120.]])), axis=1)
        self.Vcomp = np.ones((3,timesteps),dtype=np.complex_)
        self.Icomp = np.ones((3,timesteps),dtype=np.complex_)


class connector:
# Includes lines, transformers, and switches as subclasses
    def __init__(self,timesteps):
        self.phasevec = np.zeros((3,1))
        self.Imag_NL = np.zeros((3,timesteps))
        self.Iang_NL = np.array([[0.],[0.],[0.]])
        for idx in range(timesteps-1):
            self.Iang_NL = np.concatenate((self.Iang_NL,np.array([[0.],[0.],[0.]])), axis=1)

class Vsrc:
# These are defined and populated, but as of 7/31/2018 they are not used.
    def __init__(self,desig):
        self.name = 'Vsrc_' + str(desig)
        self.id = ''
        self.phases = list()
        self.kV = complex(0)
        self.angle_phA = complex(0)

        self.phasevec = np.zeros((3,1))
        self.Vvec_phg = np.zeros((3,1))
        self.Vangvec = np.zeros((3,1))

class shunt:
# As of 7/31/2018, this class only supports cap banks.
    def __init__(self,desig,timesteps):
        self.name = 'cap_' + str(desig)
        self.id = ''
        self.node = None
        self.phases = list()
        self.phasevec = np.zeros((3,1))
        self.kV_phg = complex(0)

        self.phasevec = np.zeros((3,1))
        self.Pvec = np.zeros((3,1))
        self.Qvec = np.zeros((3,1))
        self.statusvec = np.zeros((3,1))

        self.P_NL = np.zeros((3,timesteps))
        self.Q_NL = np.zeros((3,timesteps))


class load:
    def __init__(self, desig, timesteps):
        self.name = 'load_' + str(desig)
        self.id = ''
        self.type = ''
        self.node = None
        self.phases = list()
        self.phasevec = np.zeros((3,1))
        self.kV_phph = complex(0)
        self.bandwidth = complex(0)
        self.conn = ''
        self.constZ = complex(0)
        self.constI = complex(0)
        self.constP = complex(0)
        self.status = list()

        self.Rsched = np.zeros((3,timesteps))
        self.Xsched = np.zeros((3,timesteps))
        self.Psched = np.zeros((3,timesteps))
        self.Qsched = np.zeros((3,timesteps))

        self.P_NL = np.zeros((3,timesteps))
        self.Q_NL = np.zeros((3,timesteps))

class actuator:
# As of 7/31/2018, only set up to handle one actuator per node
    def __init__(self, desig, timesteps):
        self.name = 'act_' + str(desig)
        self.node = None
        self.phases = list()
        self.phasevec = np.zeros((3,1))

        #not sure how/if these are used
        self.Psched = np.zeros((3,timesteps))
        self.Ssched = np.zeros((3,timesteps))
        #self.Qsched = np.zeros((3,timesteps))

        self.Pgen = np.zeros((3,timesteps)) #changed from cp
        self.Qgen = np.zeros((3,timesteps))

class line(connector):
    def __init__(self, desig, timesteps):
        connector.__init__(self,timesteps)
        self.name = 'line_' + str(desig)
        self.id = ''

        self.from_node = None
        self.from_phases = list()
        self.to_node = None
        self.to_phases = list() # think these should always be equal to from
        self.length = complex(0)
        self.MVArating_3ph = complex(0)
        self.kVbase_phg = complex(0)
        self.kVAbase = complex(0)
        self.Zbase = complex(0)

        self.R = np.zeros((3, 3))
        self.X = np.zeros((3, 3))
        self.shuntB = np.zeros((3, 3), dtype=np.complex_)

        self.Z = np.zeros((3, 3), dtype=np.complex_)
        self.Y = np.zeros((3, 3), dtype=np.complex_)

        self.Zpu = np.zeros((3, 3), dtype=np.complex_)
        self.Ypu = np.zeros((3, 3), dtype=np.complex_)

        self.P_lin = np.zeros((3,timesteps)) #changed from cp
        self.Q_lin = np.zeros((3,timesteps))

class switch(connector):
    def __init__(self,desig,timesteps):
        connector.__init__(self,timesteps)
        self.name = 'switch_' + str(desig)
        self.phases_from = list()
        self.phases_to = list()
        self.from_node = None
        self.to_node = None
        self.status = list()

        self.Z = np.zeros((3, 3), dtype=np.complex_)
        self.Y = np.zeros((3, 3), dtype=np.complex_)
        self.Zpu = np.zeros((3, 3), dtype=np.complex_)
        self.Ypu = np.zeros((3, 3), dtype=np.complex_)

        self.P_lin = np.zeros((3,timesteps)) #changed from cp
        self.Q_lin = np.zeros((3,timesteps))

class transformer(connector):
    def __init__(self,desig,timesteps):
        connector.__init__(self,timesteps)
        self.name = 'transformer_' + str(desig)
        self.id = ''

        self.w0_node = None
        self.w0_phases = list()
        self.w0_kVbase_phg = complex(0)
        self.w0_kVAbase = complex(0)
        self.w0_rpu = complex(0)
        self.w0_conn = ''
        self.w1_node = None
        self.w1_phases = list()
        self.w1_kVbase_phg = complex(0)
        self.w1_kVAbase = complex(0)
        self.w1_rpu = complex(0)
        self.w1_conn = ''

        self.xpu = complex(0)
        self.tappos = list()
        self.taprange_high = complex(0)
        self.taprange_low = complex(0)
        self.taprangepct_high = complex(0)
        self.taprangepct_low = complex(0)

        self.Zpu = np.zeros((3, 3), dtype=np.complex_)
        self.Ypu = np.zeros((3, 3), dtype=np.complex_)

        self.P_lin = np.zeros((3,timesteps)) #changed from cp
        self.Q_lin = np.zeros((3,timesteps))


# In[27]:


def phase2vec(letter):
    # Converts 'a','b','c' to elementary vectors e1, e2, or e3
    assert (letter == 'a' or letter == 'b' or letter == 'c'), "Bad call to phase2vec"
    vecout = np.zeros((3,1))
    if letter == 'a':
        vecout = vecout + np.array([[1],[0],[0]])
    elif letter == 'b':
        vecout = vecout + np.array([[0],[1],[0]])
    elif letter == 'c':
        vecout = vecout + np.array([[0],[0],[1]])
    return vecout

def ZtoY(Zmat):
    # Converts an impedance matrix to an admittance matrix
    # Treats '0' impedances on the diagonal as 'line not present' (i.e. temporarily assigns them Z=infinite before inverting)
    tempZ = deepcopy(Zmat)
    for idx in range(0,3):
        if Zmat[idx,idx] == 0:
            tempZ[idx,idx] = np.inf
    return np.linalg.pinv(tempZ) # changed this

def fixconnections(tree,headnode,assignednodes=[]):
    # Removes all upstream-facing connections from a networkx digraph
    succlist = list()
    assignlist = assignednodes
    assignlist.append(headnode)
    for inode in tree.successors(headnode):
        succlist.append(inode)
    for inode in succlist:
        if all(elem in assignlist for elem in succlist):
            return        
        if inode in assignlist:
            continue
        else:
            if (inode, headnode) in tree.edges():
                tree.remove_edge(inode,headnode)
            fixconnections(tree,inode,assignlist)

def propogatebasesup(tree,currnode,kVbase_phg,assignednodes=[]):
    # Sets the voltage bases of every node on a path upstream from a given node. Stops if it hits a transformer.
    # 'tree' should be a networkx graph object            
    predlist = list()
    assignlist = assignednodes
    assignlist.append(currnode)
    currnode.kVbase_phg = kVbase_phg
    currnode.Zbase = kVbase_phg*kVbase_phg*1000/currnode.kVAbase
    for inode in tree.predecessors(currnode):
        predlist.append(inode)
    for inode in predlist:
        if isinstance(tree[inode][currnode]['connector'],transformer):
            return
        if all(elem in assignlist for elem in predlist):
            return
        if inode in assignlist:
            continue
        else:
            propogatebasesup(tree,inode,kVbase_phg,assignlist)
        
def propogatebasesdown(tree,currnode,kVbase_phg,assignednodes=[]):
    # Sets the voltage bases of every node on a path downtream from a given node. Stops if it hits a transformer.
    # 'tree' should be a networkx graph object  
    succlist = list()   
    assignlist = assignednodes
    assignlist.append(currnode)
    currnode.kVbase_phg = kVbase_phg
    currnode.Zbase = kVbase_phg*kVbase_phg*1000/currnode.kVAbase
    for inode in tree.successors(currnode):
        succlist.append(inode)
    for inode in succlist:
        if all(elem in assignlist for elem in succlist):
            return
        if inode in assignlist:
            continue
        else:
            propogatebasesdown(tree,inode,kVbase_phg,assignlist)
        
def propogatetrans(tree,currnode,assignednodes=[]):
    # Uses propogatebasesup and down to set appropriate voltage bases throughout a networkx graph based on transformer ratings.
    edgelist = list()
    assignlist = assignednodes
    assignlist.append(currnode)
    for iedge in tree.out_edges(currnode): #2to3 out_edges_iter to out_edges
        edgelist.append(iedge)
    for iedge in edgelist:
        conn = tree[iedge[0]][iedge[1]]['connector']
        if isinstance(conn,transformer):
            if currnode == conn.w0_node:
                propogatebasesup(tree,currnode,conn.w0_kVbase_phg)
                propogatebasesdown(tree,iedge[1],conn.w1_kVbase_phg)
            elif currnode == conn.w1_node:
                propogatebasesup(tree,currnode,conn.w1_kVbase_phg)
                propogatebasesdown(tree,iedge[1],conn.w0_kVbase_phg)
    succlist = list()
    for inode in tree.successors(currnode):
        succlist.append(inode)
    for inode in succlist:
        if all(elem in assignlist for elem in succlist):
            return
        if inode in assignlist:
            continue
        else:
            propogatetrans(tree,inode,assignlist)


# In[28]:


## DATA IMPORT FUNCTIONS


# In[29]:


# CREATING NODES AND LOAD OBJECTS


# In[30]:


# Create bus dictionary from dataframe
def busbuilder(modeldata,subkVbase_phg,subkVAbase,timesteps):
    bussheet = modeldata.parse('Bus')
    busdict=dict()

    for idx, row in bussheet.iterrows():
        indkey = row['Bus'][:len(row['Bus'])-2]
        indphase = row['Bus'][len(row['Bus'])-1]

        if indkey in busdict.keys():
            busdict[indkey].phases.append(indphase)
        else:
            busdict[indkey] = bus(indkey,subkVbase_phg,subkVAbase, timesteps)
            busdict[indkey].type = row['Type'] #this is where one bus gets designated SLACK
            busdict[indkey].phases.append(indphase)

    #create onehot vectors for the phases of each bus
    for key, obj in busdict.items():
        for idx2 in range(len(obj.phases)):
            if obj.phases[idx2] == 'a':
                obj.phasevec = obj.phasevec + np.array([[1],[0],[0]])
            elif obj.phases[idx2] == 'b':
                obj.phasevec = obj.phasevec + np.array([[0],[1],[0]])
            elif obj.phases[idx2] == 'c':
                obj.phasevec = obj.phasevec + np.array([[0],[0],[1]])

    return busdict


# In[31]:


# Create Vsource dictionary from dataframe
# This is a superfluous object since the present (July 2018) code assumes that there is only one Vsrc (the substation)
# So, the current code doesn't use it for anything, but if later models have multiple Vsrcs, we might find it useful
def Vsrcbuilder(modeldata, busdict):
    Vsrcsheet = modeldata.parse('Vsource 3-phase')
    Vsrcdict=dict()

    # Initiation
    for idx, row in Vsrcsheet.iterrows():
        if row['bus A']:
            indkey = row['bus A'][:len(row['bus A'])-2]
        elif row['bus B']:
            indkey = row['bus B'][:len(row['bus B'])-2]
        elif row['bus C']:
            indkey = row['bus C'][:len(row['bus C'])-2] # Note that there is a leading space in the Excel sheet
        Vsrcdict[indkey] = Vsrc(indkey)

        # Check phases
        if row['bus A'] and isinstance(row['bus A'],str):
            Vsrcdict[indkey].phases.append('a')
        if row['bus B'] and isinstance(row['bus B'],str):
            Vsrcdict[indkey].phases.append('b')
        if row['bus C'] and isinstance(row['bus C'],str): # Note that there is a leading space in the Excel sheet
            Vsrcdict[indkey].phases.append('c')

        # Set angles. This assumes that "Angle A" is the only one populated, (the only form I've seen so far)
        angle_phA = row['Angle_a (Degree)']
        Vsrcdict[indkey].Vangvec[0,0] = angle_phA
        Vsrcdict[indkey].Vangvec[1,0] = angle_phA + 240.
        Vsrcdict[indkey].Vangvec[2,0] = angle_phA + 120.

        # Misc field, likely not useful
        Vsrcdict[indkey].id = row['ID']

        # Set voltages
        Vsrcdict[indkey].kV_phph = row['kV (ph-ph RMS)']
        Vsrcdict[indkey].Vvec_phg[0,0] = Vsrcdict[indkey].kV_phph/np.sqrt(3)
        Vsrcdict[indkey].Vvec_phg[1,0] = Vsrcdict[indkey].kV_phph/np.sqrt(3)
        Vsrcdict[indkey].Vvec_phg[2,0] = Vsrcdict[indkey].kV_phph/np.sqrt(3)

        # Append object to buslist
        busdict[indkey].Vsrcs.append(Vsrcdict[indkey])

    # Populate phase vectors
    for key, obj in Vsrcdict.items():
        for idx2 in range(len(obj.phases)):
            obj.phasevec = obj.phasevec + phase2vec(obj.phases[idx2])

    return Vsrcdict


# In[32]:


# Create shunt elements dictionary from dataframe
# Currently (July 2018) this object handles constant power capacitors only
def shuntbuilder(modeldata, busdict,timesteps):
    shuntsheet = modeldata.parse('Multiphase Shunt')
    shuntdict = dict()

    for idx, row in shuntsheet.iterrows():
        if row['Bus1'] and isinstance(row['Bus1'],str):
            indkey = row['Bus1'][:len(row['Bus1'])-2]
        elif row['Bus2'] and isinstance(row['Bus2'],str):
            indkey = row['Bus2'][:len(row['Bus2'])-2]
        elif row['Bus3'] and isinstance(row['Bus3'],str):
            indkey = row['Bus3'][:len(row['Bus3'])-2]

        shuntdict[indkey] = shunt(indkey,timesteps)

        if row['Bus1'] and isinstance(row['Bus1'],str):
            indphase = row['Bus1'][len(row['Bus1'])-1]
            shuntdict[indkey].phases.append(indphase)

        if row['Bus2'] and isinstance(row['Bus2'],str):
            indphase = row['Bus2'][len(row['Bus2'])-1]
            shuntdict[indkey].phases.append(indphase)

        if row['Bus3'] and isinstance(row['Bus3'],str):
            indphase = row['Bus3'][len(row['Bus3'])-1]
            shuntdict[indkey].phases.append(indphase)

        shuntdict[indkey].id = row['ID']
        shuntdict[indkey].node = busdict[indkey]
        shuntdict[indkey].kV_phg = row['kV (ph-gr RMS)']

        #KEITH changed long to int
        if (row['P1 (kW)']) and (not np.isnan(row['P1 (kW)'])) and (isinstance(row['P1 (kW)'],int) or isinstance(row['P1 (kW)'],float)):
            # Note that spacings in P,Q headings are very inconsistent
            shuntdict[indkey].Pvec = shuntdict[indkey].Pvec + row['P1 (kW)'] * phase2vec(shuntdict[indkey].phases[0])

        if row['P2 (kW)'] and (not np.isnan(row['P2 (kW)'])) and (isinstance(row['P2 (kW)'],int) or isinstance(row['P2 (kW)'],float)):
            shuntdict[indkey].Pvec = shuntdict[indkey].Pvec + row['P2 (kW)'] * phase2vec(shuntdict[indkey].phases[1])

        if row['P3 (kW)'] and (not np.isnan(row['P3 (kW)'])) and (isinstance(row['P3 (kW)'],int) or isinstance(row['P3 (kW)'],float)):
            shuntdict[indkey].Pvec = shuntdict[indkey].Pvec + row['P3 (kW)'] * phase2vec(shuntdict[indkey].phases[2])

        if row['Q1 (kVAr)'] and (not np.isnan(row['Q1 (kVAr)'])) and (isinstance(row['Q1 (kVAr)'],int) or isinstance(row['Q1 (kVAr)'],float)):
            shuntdict[indkey].Qvec = shuntdict[indkey].Qvec + row['Q1 (kVAr)'] * phase2vec(shuntdict[indkey].phases[0])

        if row['Q2 (kVAr)'] and (not np.isnan(row['Q2 (kVAr)'])) and (isinstance(row['Q2 (kVAr)'],int) or isinstance(row['Q2 (kVAr)'],float)):
            shuntdict[indkey].Qvec = shuntdict[indkey].Qvec + row['Q2 (kVAr)'] * phase2vec(shuntdict[indkey].phases[1])

        if row['Q3 (kVAr)'] and (not np.isnan(row['Q3 (kVAr)'])) and (isinstance(row['Q3 (kVAr)'],int) or isinstance(row['Q3 (kVAr)'],float)):
            shuntdict[indkey].Qvec = shuntdict[indkey].Qvec + row['Q3 (kVAr)'] * phase2vec(shuntdict[indkey].phases[2])

        if row['Status1'] and row['Status1'] == 1:
            shuntdict[indkey].statusvec = shuntdict[indkey].statusvec + phase2vec(shuntdict[indkey].phases[0])
        if row['Status2'] and row['Status2'] == 1:
            assert(len(shuntdict[indkey].phases) >= 2)
            shuntdict[indkey].statusvec = shuntdict[indkey].statusvec + phase2vec(shuntdict[indkey].phases[1])
        if row['Status3'] and row['Status3'] == 1:
            assert(len(shuntdict[indkey].phases) == 3)
            shuntdict[indkey].statusvec = shuntdict[indkey].statusvec + phase2vec(shuntdict[indkey].phases[2])

        busdict[indkey].cap.append(shuntdict[indkey])

    for key, obj in shuntdict.items():
        for idx2 in range(len(obj.phases)):
            obj.phasevec = obj.phasevec + phase2vec(obj.phases[idx2])

    return shuntdict


# In[33]:


# Create load dictionary from ePHASORsim model and Gridbright load files
# Create load dictionary from ePHASORsim model and Gridbright load files
def loadbuilderPQ(modeldata, busdict, loadpath, timesteps, timestepcur = 11*60):
    
    # This creates the load objects from the ePHASORsim model, but ignores the specified values (values are set with a Gridbright load file)
    loadsheet = modeldata.parse('Multiphase Load')
    loaddict = dict()
    
    for idx, row in loadsheet.iterrows():          
        if row['Bus1']:
            indkey = row['Bus1'][:len(row['Bus1'])-2]
            loadname = row['Bus1']
        elif row['Bus2']:
            indkey = row['Bus2'][:len(row['Bus2'])-2]
            loadname = row['Bus2']
        elif row['Bus3']:
            indkey = row['Bus3'][:len(row['Bus3'])-2]
            loadname = row['Bus3']

        loaddict[indkey] = load(indkey, timesteps)
        if row['Bus1'] and isinstance(row['Bus1'],str):
            indphase = row['Bus1'][len(row['Bus1'])-1]
            loaddict[indkey].phases.append(indphase)

        if row['Bus2'] and isinstance(row['Bus2'],str):
            indphase = row['Bus2'][len(row['Bus2'])-1]
            loaddict[indkey].phases.append(indphase)

        if row['Bus3'] and isinstance(row['Bus3'],str):
            indphase = row['Bus3'][len(row['Bus3'])-1]
            loaddict[indkey].phases.append(indphase)
        
        loaddict[indkey].id = row['ID']
        loaddict[indkey].node = busdict[indkey]
        loaddict[indkey].type = row['Type']
        loaddict[indkey].kV_phph = row['V (kV)']
        loaddict[indkey].bandwidth = row['Bandwidth (pu)']
        loaddict[indkey].conn = row['Conn. type']
        loaddict[indkey].constZ = row['K_z']
        loaddict[indkey].constI = row['K_i']
        loaddict[indkey].constP = row['K_p']
        loaddict[indkey].status = row['Status']
        
        
        busdict[indkey].loads.append(loaddict[indkey])
        
    loadfile = pd.ExcelFile(loadpath)
    loaddf = loadfile.parse('Time_Series_data')
    #[HIL] - parse out current timestep
    loaddf = loaddf[timestepcur:timestepcur+timesteps]
    loaddf.index -= timestepcur
    
    # scale load
    ld_scale = 0.3
    if ld_scale != 1 == True:
        print(f'*** WARNING - load scale = {ld_scale} ***')
    loaddf = loaddf * ld_scale
    
    

    # Populate the load dictionary's P and Q schedules with a Gridbright load file
    
#JPEDIT START
    
    multiph = ['1','2','3'] ##generalize to 1,2,3 instead of first second third
    for key, iload in loaddict.items():
        Pkey = []
        Qkey = []
        phkey = []
        for header in loaddf.columns:
            if f'LD_{key}/' in header:
                #for mph in multiph:
                if '/P' in header:
                    Pkey.append(header)
                if '/Q' in header:
                    Qkey.append(header)
        for ph in iload.phases:
            phkey.append(ph)
        for idx in range(len(phkey)):
            for ts in range(0,timesteps):  #write with dict??
                if idx+1 > len(Pkey):
                    kW = 0
                    kVAR = 0
                else: 
                    kW = loaddf[Pkey[idx]][ts]
                    kVAR = loaddf[Qkey[idx]][ts]
                    
                S = kW + 1j*kVAR
                    
                if not (S==0):
                    Z = np.conj(np.square(iload.node.kVbase_phg)*1000/S)
                else:
                    Z = 0
                
                if phkey[idx] == 'a':
                    iload.Psched[0,ts] = kW
                    iload.Qsched[0,ts] = kVAR
    
                    iload.Rsched[0,ts] = np.real(Z)
                    iload.Xsched[0,ts] = np.imag(Z)
    
                if phkey[idx] == 'b':
                    iload.Psched[1,ts] = kW
                    iload.Qsched[1,ts] = kVAR
    
                    iload.Rsched[1,ts] = np.real(Z)
                    iload.Xsched[1,ts] = np.imag(Z)
    
                if phkey[idx] == 'c':
                    iload.Psched[2,ts] = kW
                    iload.Qsched[2,ts] = kVAR
    
                    iload.Rsched[2,ts] = np.real(Z)
                    iload.Xsched[2,ts] = np.imag(Z)
                
#JPEDIT END - replaced edited out code, see old backup versions if this needs to be restored

    for key, iload in loaddict.items():
        for idx in range(len(iload.phases)):
            if iload.phases[idx] == 'a':
                iload.phasevec = iload.phasevec + np.array([[1],[0],[0]])
            elif iload.phases[idx] == 'b':
                iload.phasevec = iload.phasevec + np.array([[0],[1],[0]])
            elif iload.phases[idx] == 'c':
                iload.phasevec = iload.phasevec + np.array([[0],[0],[1]])
            
    return loaddict


# In[34]:


# Create actuator dictionary from Gridbright load file
# This interprets all 'PV_KW_' entries in the load file as controllable PV.
# If the PV is uncontrollable, it should be included as negative load in the load P and Q schedules
def actbuilder(modeldata, busdict, actpath, timesteps):

    actdict = dict()
    actfile = pd.ExcelFile(actpath) #actpath is loadpath (for now), so Psched and Qsched come from loadfile
    actdf = actfile.parse('Time_Series_data')

    for key,ibus in busdict.items():
        for ph in ibus.phases:
            if 'PV_KW_' + key + '_' + ph in list(actdf.columns.values):
                if not (key in actdict):
                    actdict[key] = actuator(key, timesteps)
                    actdict[key].node = busdict[key]
                    ibus.actuators.append(actdict[key])
                actdict[key].phases.append(ph)

    for key, iact in actdict.items():
        for ph in iact.phases:
            for ts in range(0,timesteps):
                actname_PV = 'PV_KW_' + key + '_' + ph
                PV = actdf[actname_PV][ts]
                if ph == 'a':
                    iact.Psched[0,ts] = PV
                    iact.Ssched[0,ts] = PV
                if ph == 'b':
                    iact.Psched[1,ts] = PV
                    iact.Ssched[1,ts] = PV
                if ph == 'c':
                    iact.Psched[2,ts] = PV
                    iact.Ssched[2,ts] = PV

    for key, iact in actdict.items():
        for idx in range(len(iact.phases)):
            if iact.phases[idx] == 'a':
                iact.phasevec = iact.phasevec + np.array([[1],[0],[0]])
            elif iact.phases[idx] == 'b':
                iact.phasevec = iact.phasevec + np.array([[0],[1],[0]])
            elif iact.phases[idx] == 'c':
                iact.phasevec = iact.phasevec + np.array([[0],[0],[1]])

    return actdict


# In[35]:


# LINES AND CONNECTOR OBJECTS


# In[36]:


# Create line dictionary
# Note that this isn't set up to handle cross-phase connections: i.e. please don't connect phase A at node 1 to phase B at node 2
def linebuilder(modeldata, busdict, timesteps):
    linesheet = modeldata.parse('Multiphase Line')
    # Create line dictionary from dataframe
    linedict = dict()
    for idx, row in linesheet.iterrows():
        if row['From1']:
            indkeyfrom = row['From1'][:len(row['From1'])-2]
        elif row['From2']:
            indkeyfrom = row['From2'][:len(row['From2'])-2]
        elif row['From3']:
            indkeyfrom = row['From3'][:len(row['From3'])-2]

        if row['To1']:
            indkeyto = row['To1'][:len(row['To1'])-2]
        elif row['To2']:
            indkeyto = row['To2'][:len(row['To2'])-2]
        elif row['To3']:
            indkeyto = row['To3'][:len(row['To3'])-2]

        indkey = indkeyfrom + 'to' + indkeyto
        linedict[indkey] = line(indkey, timesteps)

        busdict[indkeyfrom].edges_out.append(linedict[indkey])
        busdict[indkeyto].edges_in.append(linedict[indkey])

        linedict[indkey].id = row['ID']
        linedict[indkey].length = row['Length (length_unit)']
        
        # [Jasper] - capacity edit
        if 'MVA rating' in linesheet.columns:
            if row['MVA rating']:
                linedict[indkey].MVArating_3ph = row['MVA rating']

        linedict[indkey].from_node = busdict[indkeyfrom]
        linedict[indkey].to_node = busdict[indkeyto]

        if row['From1'] and isinstance(row['From1'],str):
            linedict[indkey].from_phases.append(row['From1'][len(row['From1'])-1])
        if row['From2'] and isinstance(row['From2'],str):
            linedict[indkey].from_phases.append(row['From2'][len(row['From2'])-1])
        if row['From3'] and isinstance(row['From3'],str):
            linedict[indkey].from_phases.append(row['From3'][len(row['From3'])-1])

        if row['To1'] and isinstance(row['To1'],str):
            linedict[indkey].to_phases.append(row['To1'][len(row['To1'])-1])
        if row['To2'] and isinstance(row['To2'],str):
            linedict[indkey].to_phases.append(row['To2'][len(row['To2'])-1])
        if row['To3'] and isinstance(row['To3'],str):
            linedict[indkey].to_phases.append(row['To3'][len(row['To3'])-1])

        for idx in range(0,len(linedict[indkey].to_phases)):
            linedict[indkey].phasevec = linedict[indkey].phasevec + phase2vec(linedict[indkey].to_phases[idx])

        # Build R, X, shunt Y matrices (shunt Y are not included in LUPFM)
        #line impedance matrices are not impedance values for each line (??)
        rdict = dict()
        rdict['11'] = row['r11 (ohm/length_unit)']
        rdict['21'] = row['r21 (ohm/length_unit)']
        rdict['22'] = row['r22 (ohm/length_unit)']
        rdict['31'] = row['r31 (ohm/length_unit)']
        rdict['32'] = row['r32 (ohm/length_unit)']
        rdict['33'] = row['r33 (ohm/length_unit)']
        rdict['12'] = rdict['21']
        rdict['13'] = rdict['31']
        rdict['23'] = rdict['32']

        xdict = dict()
        xdict['11'] = row['x11 (ohm/length_unit)']
        xdict['21'] = row['x21 (ohm/length_unit)']
        xdict['22'] = row['x22 (ohm/length_unit)']
        xdict['31'] = row['x31 (ohm/length_unit)']
        xdict['32'] = row['x32 (ohm/length_unit)']
        xdict['33'] = row['x33 (ohm/length_unit)']
        xdict['12'] = xdict['21']
        xdict['13'] = xdict['31']
        xdict['23'] = xdict['32']

        bdict = dict()
        bdict['11'] = row['b11 (uS/length_unit)']
        bdict['21'] = row['b21 (uS/length_unit)']
        bdict['22'] = row['b22 (uS/length_unit)']
        bdict['31'] = row['b31 (uS/length_unit)']
        bdict['32'] = row['b32 (uS/length_unit)']
        bdict['33'] = row['b33 (uS/length_unit)']
        bdict['12'] = bdict['21']
        bdict['13'] = bdict['31']
        bdict['23'] = bdict['32']

        frm_ph_lst = list(map(string.ascii_lowercase.index,linedict[indkey].from_phases))
        to_ph_lst = list(map(string.ascii_lowercase.index,linedict[indkey].to_phases))
        for fromidx in range(0, len(frm_ph_lst)):
            for toidx in range(0, len(to_ph_lst)):
                linedict[indkey].R[frm_ph_lst[fromidx], to_ph_lst[toidx]] = rdict[str(fromidx + 1) + str(toidx + 1)] #puts the resistances (and admittance) into R one-by-bone
                linedict[indkey].R[to_ph_lst[toidx], frm_ph_lst[fromidx]] = rdict[str(fromidx + 1) + str(toidx + 1)]
                linedict[indkey].X[to_ph_lst[toidx], frm_ph_lst[fromidx]] = xdict[str(fromidx + 1) + str(toidx + 1)]
                linedict[indkey].X[frm_ph_lst[fromidx], to_ph_lst[toidx]] = xdict[str(fromidx + 1) + str(toidx + 1)]
        linedict[indkey].R = np.nan_to_num(np.multiply(linedict[indkey].R,linedict[indkey].length)) #converts per length values to non pu Rs and Xs
        linedict[indkey].X = np.nan_to_num(np.multiply(linedict[indkey].X,linedict[indkey].length))

        linedict[indkey].Z = linedict[indkey].R + 1j*linedict[indkey].X
        linedict[indkey].Y = ZtoY(linedict[indkey].Z)
        # Line per-unit quantities are set later in this script, once voltage bases have been established.

    return linedict


# In[37]:


# Create switch dictionary
def switchbuilder(modeldata, busdict, timesteps):
    switchsheet = modeldata.parse('Switch')

    switchdict = dict()
    for idx, row in switchsheet.iterrows():
        #if row['Status'] != 0: #only create node/edge if switch is closed
        indkeyfrom = row['From Bus'][:len(row['From Bus'])-2]
        indkeyto = row['To Bus'][:len(row['To Bus'])-2]

        indphasefrom = row['From Bus'][len(row['From Bus'])-1]
        indphaseto = row['To Bus'][len(row['To Bus'])-1]

        indkey = indkeyfrom + 'to' + indkeyto

        if indkey in switchdict.keys():
            switchdict[indkey].phases_from.append(indphasefrom)
            switchdict[indkey].phases_to.append(indphaseto)

        else:
            switchdict[indkey] = switch(indkey, timesteps)
            switchdict[indkey].phases_from.append(indphasefrom)
            switchdict[indkey].phases_to.append(indphaseto)

            switchdict[indkey].from_node = busdict[indkeyfrom]
            switchdict[indkey].to_node = busdict[indkeyto]
            switchdict[indkey].status.append(row['Status'])

            busdict[indkeyfrom].edges_out.append(switchdict[indkey])
            busdict[indkeyto].edges_in.append(switchdict[indkey])

    # This next loop is ugly, but it works (it's the result of fixing a bug)
    for indkey, entry in switchdict.items():
        for idx in range(0,len(switchdict[indkey].phases_to)):
            switchdict[indkey].phasevec = switchdict[indkey].phasevec + phase2vec(switchdict[indkey].phases_to[idx])

        # Build Z, Y matrices (per unit)
        assert(switchdict[indkey].from_node.Zbase == switchdict[indkey].to_node.Zbase)
        Zbase = switchdict[indkey].from_node.Zbase

        tempz = (1e-4)/Zbase # Based off of the DSS switch approximation: r1=r0=1e-4
        tempy = 1/tempz

        from_ph_lst = list(map(string.ascii_lowercase.index,switchdict[indkey].phases_from))
        to_ph_lst = list(map(string.ascii_lowercase.index,switchdict[indkey].phases_to))

        for idx in range(0, len(from_ph_lst)):
            switchdict[indkey].Z[from_ph_lst[idx], to_ph_lst[idx]] = 1e-4 #switch impedance set to a default here
            switchdict[indkey].Y[from_ph_lst[idx], to_ph_lst[idx]] = 1e4
            switchdict[indkey].Zpu[from_ph_lst[idx], to_ph_lst[idx]] = tempz
            switchdict[indkey].Ypu[from_ph_lst[idx], to_ph_lst[idx]] = tempy

    return switchdict


# In[38]:


# Create transformer dictionary from dataframe
def transbuilder(modeldata,busdict,subkVAbase,timesteps):
    # Multiple formats due to impedance model building process -> 3phase and Multiphase formats
    # Turn on/off the transformer mode as necessary according to the impedance model
    
    #transtype = 'multiphase' # [multiphase OR 3phase]
    transtype = '3phase' 
    
    'Transformer 3-PHASE'
    if transtype == '3phase':
        transsheet = modeldata.parse('Transformer 3-phase')
        #[HIL] - NEW CODE index misalignment during parse
        if transsheet.iloc[0][0] == 'ID':
            transsheet = modeldata.parse('Transformer 3-phase', index_col=0)
        
        # Prep transformer column headers (This is built to handle a 2-winding transformer)
        windcols = transsheet.columns.get_loc('winding 1') - transsheet.columns.get_loc('winding 0')
        othercols = len(transsheet.columns) - transsheet.columns.get_loc('winding 1') - windcols
        for idx in range(0,windcols):
            transsheet.iloc[0][idx] = 'w0_' + transsheet.iloc[0][idx]
        for idx in range(windcols,2*windcols):
            transsheet.iloc[0][idx] = 'w1_' + transsheet.iloc[0][idx]
        transsheet.columns = transsheet.iloc[0]
        transsheet = transsheet.drop(transsheet.index[0]);
        
        # Create dictionary
        transdict = dict()
        
        for idx, row in transsheet.iterrows():
            if row['w0_bus a']:
                indkeyw0 = row['w0_bus a'][:len(row['w0_bus a'])-2]       
            elif row['w0_bus b']:
                indkeyw0 = row['w0_bus b'][:len(row['w0_bus b'])-2]
            elif row['w0_bus c']:
                indkeyw0 = row['w0_bus c'][:len(row['w0_bus c'])-2] 
    
            if row['w1_bus_a']:
                indkeyw1 = row['w1_bus_a'][:len(row['w1_bus_a'])-2]       
            elif row['w1_bus_b']:
                indkeyw1 = row['w1_bus_b'][:len(row['w1_bus_b'])-2]
            elif row['w1_bus_c']:
                indkeyw1 = row['w1_bus_c'][:len(row['w1_bus_c'])-2]     
            
            indkey = indkeyw0 + 'to' + indkeyw1    
        
            transdict[indkey] = transformer(indkey, timesteps)
            transdict[indkey].id = idx
            
            busdict[indkeyw0].edges_out.append(transdict[indkey])
            busdict[indkeyw1].edges_in.append(transdict[indkey])
            
            if row['w0_bus a']:
                transdict[indkey].w0_phases.append('a')    
            if row['w0_bus b']:
                transdict[indkey].w0_phases.append('b')   
            if row['w0_bus c']:
                transdict[indkey].w0_phases.append('c')
            
            transdict[indkey].w0_name = indkeyw0
            transdict[indkey].w0_node = busdict[indkeyw0]
        
            transdict[indkey].w0_kVbase_phg = row['w0_kV (ph-ph RMS)']/np.sqrt(3) 
            transdict[indkey].w0_kVAbase = row['w0_kVA_base']
            transdict[indkey].w0_rpu = row['w0_R_w0 (pu)'] 
            #transdict[indkey].w0_conn = row[headmap['w0_conn']]
            transdict[indkey].w0_conn = row['w0_conn'] #[HIL] - edit, error
    
            if row['w1_bus_a']:
                transdict[indkey].w1_phases.append('a')    
            if row['w1_bus_b']:
                transdict[indkey].w1_phases.append('b')   
            if row['w1_bus_c']:
                transdict[indkey].w1_phases.append('c')
                
            for idx in range(0,len(transdict[indkey].w1_phases)):
                transdict[indkey].phasevec = transdict[indkey].phasevec + phase2vec(transdict[indkey].w1_phases[idx])
    
            transdict[indkey].w1_name = indkeyw1
            transdict[indkey].w1_node = busdict[indkeyw1]
        
            transdict[indkey].w1_kVbase_phg = row['w1_kV (ph-ph RMS)']/np.sqrt(3) 
            transdict[indkey].w1_kVAbase = row['w1_kVA_base'] 
            transdict[indkey].w1_rpu = row['w1_R_w1 (pu)'] 
            transdict[indkey].w1_conn = row['w1_conn'] 
            
            transdict[indkey].xpu = row['X (pu)'] 
    
            if row['Tap A']:
                transdict[indkey].tappos.append(row['Tap A'])
            else:
                transdict[indkey].tappos.append(0)
            if row['Tap B']:
                transdict[indkey].tappos.append(row['Tap B'])
            else:
                transdict[indkey].tappos.append(0)
            if row['Tap C']:
                transdict[indkey].tappos.append(row['Tap C'])
            else:
                transdict[indkey].tappos.append(0)
        
            transdict[indkey].taprange_high = row['Highest Tap'] 
            transdict[indkey].taprange_low = row['Lowest Tap']
            transdict[indkey].taprangepct_high = row['Max Range (%)']
            transdict[indkey].taprangepct_low = row['Min Range (%)']
            
            # Build Z, Y matrices
            assert(transdict[indkey].w1_kVAbase == transdict[indkey].w0_kVAbase) 
            tempz = (transdict[indkey].w0_rpu + transdict[indkey].w1_rpu + 1j*transdict[indkey].xpu)*subkVAbase/transdict[indkey].w0_kVAbase
            tempy = 1/tempz
    
        
            w0_ph_lst = list(map(string.ascii_lowercase.index,transdict[indkey].w0_phases))
            w1_ph_lst = list(map(string.ascii_lowercase.index,transdict[indkey].w1_phases))
        
            for idx in range(0, len(w0_ph_lst)):
                transdict[indkey].Zpu[w0_ph_lst[idx], w1_ph_lst[idx]] = tempz
                transdict[indkey].Ypu[w0_ph_lst[idx], w1_ph_lst[idx]] = tempy
                
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~  MULTIPHASE  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #                
                
    'Transformer MULTIPHASE'
    if transtype == 'multiphase':
        transsheet = modeldata.parse('Multiphase Transformer')
        #[HIL] - NEW CODE index misalignment during parse
# =============================================================================
#         if transsheet.iloc[0][0] == 'ID':
#             transsheet = modeldata.parse('Transformer 3-phase', index_col=0)
#         
#         # Prep transformer column headers (This is built to handle a 2-winding transformer)
#         windcols = transsheet.columns.get_loc('winding 1') - transsheet.columns.get_loc('winding 0')
#         othercols = len(transsheet.columns) - transsheet.columns.get_loc('winding 1') - windcols
#         for idx in range(0,windcols):
#             transsheet.iloc[0][idx] = 'w0_' + transsheet.iloc[0][idx]
#         for idx in range(windcols,2*windcols):
#             transsheet.iloc[0][idx] = 'w1_' + transsheet.iloc[0][idx]
#         transsheet.columns = transsheet.iloc[0]
#         transsheet = transsheet.drop(transsheet.index[0]);
# =============================================================================
        
        # Create dictionary
        transdict = dict()
        
        for idx, row in transsheet.iterrows():
            if row['W1Bus 1']:
                indkeyw0 = row['W1Bus 1'][:len(row['W1Bus 1'])-2]       
            elif row['W1Bus 2']:
                indkeyw0 = row['W1Bus 2'][:len(row['W1Bus 2'])-2]
            elif row['W1Bus 3']:
                indkeyw0 = row['W1Bus 3'][:len(row['W1Bus 3'])-2] 
    
            if row['W2Bus 1']:
                indkeyw1 = row['W2Bus 1'][:len(row['W2Bus 1'])-2]       
            elif row['W2Bus 2']:
                indkeyw1 = row['W2Bus 2'][:len(row['W2Bus 2'])-2]
            elif row['W2Bus 3']:
                indkeyw1 = row['W2Bus 3'][:len(row['W2Bus 3'])-2]     
            
            indkey = indkeyw0 + 'to' + indkeyw1    
        
            transdict[indkey] = transformer(indkey, timesteps)
            transdict[indkey].id = idx
            
            busdict[indkeyw0].edges_out.append(transdict[indkey])
            busdict[indkeyw1].edges_in.append(transdict[indkey])

            # phases (a,b,c)
            if row['W1Bus 1'] and isinstance(row['W1Bus 1'],str):
                transdict[indkey].w0_phases.append(row['W1Bus 1'][len(row['W1Bus 1'])-1])    
            if row['W1Bus 2'] and isinstance(row['W1Bus 2'],str):
                transdict[indkey].w0_phases.append(row['W1Bus 2'][len(row['W1Bus 2'])-1])   
            if row['W1Bus 3'] and isinstance(row['W1Bus 3'],str):
                transdict[indkey].w0_phases.append(row['W1Bus 3'][len(row['W1Bus 3'])-1]) 
    
            if row['W2Bus 1'] and isinstance(row['W2Bus 1'],str):
                transdict[indkey].w1_phases.append(row['W2Bus 1'][len(row['W2Bus 1'])-1])    
            if row['W2Bus 2'] and isinstance(row['W2Bus 2'],str):
                transdict[indkey].w1_phases.append(row['W2Bus 2'][len(row['W2Bus 2'])-1])   
            if row['W2Bus 3'] and isinstance(row['W2Bus 3'],str):
                transdict[indkey].w1_phases.append(row['W2Bus 3'][len(row['W2Bus 3'])-1])
            
            transdict[indkey].w0_name = indkeyw0
            transdict[indkey].w0_node = busdict[indkeyw0]
        
            transdict[indkey].w0_kVbase_phg = row['W1V (kV)']/np.sqrt(3) 
            transdict[indkey].w0_kVAbase = row['W1S_base (kVA)']
            transdict[indkey].w0_rpu = row['W1R (pu)'] 
            transdict[indkey].w0_conn = row['W1Conn. type']
                
            for idx in range(0,len(transdict[indkey].w1_phases)):
                transdict[indkey].phasevec = transdict[indkey].phasevec + phase2vec(transdict[indkey].w1_phases[idx])
    
            transdict[indkey].w1_name = indkeyw1
            transdict[indkey].w1_node = busdict[indkeyw1]
        
            transdict[indkey].w1_kVbase_phg = row['W2V (kV)']/np.sqrt(3) 
            transdict[indkey].w1_kVAbase = row['W2S_base (kVA)']
            transdict[indkey].w1_rpu = row['W2R (pu)'] 
            transdict[indkey].w1_conn = row['W2Conn. type']
            
            transdict[indkey].xpu = row['X (pu)'] 
    
            if row['Tap 1']:
                transdict[indkey].tappos.append(row['Tap 1'])
            else:
                transdict[indkey].tappos.append(0)
            if row['Tap 2']:
                transdict[indkey].tappos.append(row['Tap 2'])
            else:
                transdict[indkey].tappos.append(0)
            if row['Tap 3']:
                transdict[indkey].tappos.append(row['Tap 3'])
            else:
                transdict[indkey].tappos.append(0)
        
            transdict[indkey].taprange_high = row['Highest Tap'] 
            transdict[indkey].taprange_low = row['Lowest Tap']
            transdict[indkey].taprangepct_high = row['Max Range (%)']
            transdict[indkey].taprangepct_low = row['Min Range (%)']
            
            # Build Z, Y matrices
            assert(transdict[indkey].w1_kVAbase == transdict[indkey].w0_kVAbase) 
            tempz = (transdict[indkey].w0_rpu + transdict[indkey].w1_rpu + 1j*transdict[indkey].xpu)*subkVAbase/transdict[indkey].w0_kVAbase
            tempy = 1/tempz
    
        
            w0_ph_lst = list(map(string.ascii_lowercase.index,transdict[indkey].w0_phases))
            w1_ph_lst = list(map(string.ascii_lowercase.index,transdict[indkey].w1_phases))
        
            for idx in range(0, len(w0_ph_lst)):
                transdict[indkey].Zpu[w0_ph_lst[idx], w1_ph_lst[idx]] = tempz
                transdict[indkey].Ypu[w0_ph_lst[idx], w1_ph_lst[idx]] = tempy
            
    return transdict


# In[39]:


## NETWORK MAP (USED TO DEFINE VOLTAGE BASES FROM TRANSFORMER CONNECTIVITY)


# In[40]:


def network_mapper(modeldata,busdict,linedict,transdict,switchdict):
    # Create networkx digraph with all connections pointing both ways
    network = nx.DiGraph()
    for key, value in busdict.items():
        network.add_node(value)
    for key, value in linedict.items():
        network.add_edge(value.from_node,value.to_node,connector=value)
        network.add_edge(value.to_node,value.from_node,connector=value)
    for key, value in transdict.items():
        network.add_edge(value.w0_node,value.w1_node,connector=value)
        network.add_edge(value.w1_node,value.w0_node,connector=value)
    for key, value in switchdict.items():
        network.add_edge(value.from_node,value.to_node,connector=value)
        network.add_edge(value.to_node,value.from_node,connector=value)

    # Find slack bus, remove all upstream-facing connections with fixconnections function, and assign kV bases
    for inode in list(network):
        if inode.type == 'SLACK' or inode.type == 'Slack' or inode.type == 'slack':
            slacknode = inode
    
    fixconnections(network,slacknode)
    propogatetrans(network,slacknode)
    
    return network


# In[41]:


def set_per_unit(linedict):
    # Set the values of Zpu and Ypu matrices for lines
    for key, iconn in linedict.items():
        if iconn.from_node.kVbase_phg != iconn.to_node.kVbase_phg:
            print('kVbase mismatch ', key, iconn.from_node.kVbase_phg, iconn.to_node.kVbase_phg)
        if iconn.from_node.kVAbase != iconn.to_node.kVAbase:
            print('kVbase mismatch ', key, iconn.from_node.kVbase_phg, iconn.to_node.kVbase_phg)
        assert (iconn.from_node.kVbase_phg == iconn.to_node.kVbase_phg and iconn.from_node.kVAbase == iconn.to_node.kVAbase), "Base mismatch " + iconn.name
        iconn.kVbase_phg = iconn.from_node.kVbase_phg
        iconn.kVAbase = iconn.from_node.kVAbase
        iconn.Zbase = iconn.kVbase_phg*iconn.kVbase_phg*1000/iconn.kVAbase
        if print_network_values == 1:
            print('*********************************************************************')
            print(f'Zbase  {iconn.Zbase}')
        iconn.Zpu = (iconn.R + 1j*iconn.X)/iconn.Zbase
        iconn.Ypu = ZtoY(iconn.Zpu)




def calc_Zeffks(Y, busidx):
    Zeffk_dict = dict()

    Ycond = 1/np.linalg.cond(Y)
    Z = np.asmatrix(np.linalg.pinv(Y, rcond=Ycond*10))

    for k, bus in enumerate(busidx):
        if k == 0: # assumes k==0 is infinite bus, dont want a Zeffk for the infinite bus (substation) and itself
            continue

        ei = np.asmatrix(np.zeros((len(Z),3)))
        e1 = np.asmatrix(np.zeros((len(Z),3)))
        ei[k*3:(k+1)*3,0:3] = np.eye(3)
        e1[0:3,0:3] = np.eye(3)
        Zeffk = np.transpose((e1 - ei))*Z*(e1 - ei)
        
        if print_network_values == 1:
            print('Zeffk for bus ' + str(bus) + ': ' + str(Zeffk))
        Zeffk_dict[bus] = Zeffk
    return (Zeffk_dict)

def export_Zeffks(Zeffk_dict, busidx, filepath):
    current_directory = filepath
    final_directory = os.path.join(current_directory, 'Zeffks_pu')
    if not os.path.exists(final_directory):
        os.makedirs(final_directory)
    os.chdir(final_directory)
    for k, bus in enumerate(busidx):
        if k == 0: # assumes k==0 is infinite bus
            continue
        #save Zeffk in a csv
        Zeffk_df = pd.DataFrame(Zeffk_dict[bus])
        Zeffk_df.to_csv('Zeffk_bus' + str(bus) + '.csv')
    os.chdir(current_directory)
    return

def export_Ybus(Ybus_pu, Ybus, filepath):
    # current_directory = filepath
    # final_directory = os.path.join(current_directory, 'Ybus')
    # if not os.path.exists(final_directory):
    #     os.makedirs(final_directory)
    # os.chdir(final_directory)
    os.chdir(filepath)
    Ybus_pu_df = pd.DataFrame(Ybus_pu)
    Ybus_df = pd.DataFrame(Ybus)
    #then save results to a csv
    Ybus_pu_df.to_csv('Ybus_pu.csv')
    Ybus_df.to_csv('Ybus.csv')
    # os.chdir(current_directory)
    return

def calc_Ybus(myfeeder):
    #buses in Y are ordered according to busdict() order, which is in order of the rows of the excel file
    #need to put the substation in the first entry
    busidx = []
    for Vsrc, bus in myfeeder.Vsrcdict.items():
        substation = Vsrc #this doest support networks with more than one substation
        if print_network_values == 1:
            print('substation at node ' + str(substation))
    busidx.append(substation)
    for key, bus in myfeeder.busdict.items():
        if key != substation:
            busidx.append(key)

    nbuses = len(busidx)
    Ypu = np.zeros((nbuses*3, nbuses*3), dtype=np.complex_)
    Y = np.zeros((nbuses*3, nbuses*3), dtype=np.complex_)

    for key, line in myfeeder.linedict.items():
        a = busidx.index(line.from_node.buskey)
        b = busidx.index(line.to_node.buskey)
        Ypu[a*3:(a+1)*3, b*3:(b+1)*3] = -np.linalg.pinv(line.Zpu)
        Ypu[b*3:(b+1)*3, a*3:(a+1)*3] = -np.linalg.pinv(line.Zpu)
        Y[a*3:(a+1)*3, b*3:(b+1)*3] = -np.linalg.pinv(line.Z)
        Y[b*3:(b+1)*3, a*3:(a+1)*3] = -np.linalg.pinv(line.Z)

    for key, xfmr in myfeeder.transdict.items():
        a = busidx.index(xfmr.w0_node.buskey)
        b = busidx.index(xfmr.w1_node.buskey)
        Ypu[a*3:(a+1)*3, b*3:(b+1)*3] = -np.reciprocal(xfmr.Zpu, out=np.zeros_like(xfmr.Zpu), where=xfmr.Zpu!=0)
        Ypu[b*3:(b+1)*3, a*3:(a+1)*3] = -np.reciprocal(xfmr.Zpu, out=np.zeros_like(xfmr.Zpu), where=xfmr.Zpu!=0)
        Zbase = myfeeder.busdict[xfmr.w0_node.buskey].Zbase #the non-pu Z of a transformer is not defined, so I just used the Zbase at w0
        Y[a*3:(a+1)*3, b*3:(b+1)*3] = -np.reciprocal(xfmr.Zpu*Zbase, out=np.zeros_like(xfmr.Zpu), where=xfmr.Zpu!=0)
        Y[b*3:(b+1)*3, a*3:(a+1)*3] = -np.reciprocal(xfmr.Zpu*Zbase, out=np.zeros_like(xfmr.Zpu), where=xfmr.Zpu!=0)

    for key, switch in myfeeder.switchdict.items():
        a = busidx.index(switch.from_node.buskey)
        b = busidx.index(switch.to_node.buskey)
        Ypu[a*3:(a+1)*3, b*3:(b+1)*3] = -np.reciprocal(switch.Zpu, out=np.zeros_like(switch.Zpu), where=switch.Zpu!=0)
        Ypu[b*3:(b+1)*3, a*3:(a+1)*3] = -np.reciprocal(switch.Zpu, out=np.zeros_like(switch.Zpu), where=switch.Zpu!=0)
        Y[a*3:(a+1)*3, b*3:(b+1)*3] = -np.reciprocal(switch.Z, out=np.zeros_like(switch.Z), where=switch.Z!=0)
        Y[b*3:(b+1)*3, a*3:(a+1)*3] = -np.reciprocal(switch.Z, out=np.zeros_like(switch.Z), where=switch.Z!=0)

    for i in np.arange(nbuses):
        rowsumYpu = np.zeros((3,3), dtype=np.complex_)
        rowsumY = np.zeros((3,3), dtype=np.complex_)
        for k in np.arange(nbuses):
            rowsumYpu = rowsumYpu + Ypu[i*3:(i+1)*3,k*3:(k+1)*3]
            rowsumY = rowsumY + Y[i*3:(i+1)*3,k*3:(k+1)*3]
        Ypu[i*3:(i+1)*3,i*3:(i+1)*3] = -rowsumYpu
        Y[i*3:(i+1)*3,i*3:(i+1)*3] = -rowsumY

    for key, shunt in myfeeder.shuntdict.items():
        pass
        #this will require making the shunt elements constant impedance, and putting them in pu
    return(Ypu, Y, busidx)
