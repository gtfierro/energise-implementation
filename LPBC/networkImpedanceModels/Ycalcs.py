

'''
Keith: I put this here to calculate the effective impedances for the LQR controller.

Its easier to have it here bc it uses setup_3.py to build a feeder model, which it then uses to calculate the admittance matrix

Doesnt take into account shunt admittances (bdict or shunt powers)
'''


import numpy as np
import sys
import pprint
# import copy

import os
from setup import *
# from dss_functions import *


def calc_Zeffk(Y, busidx, busidxs): #calc just one Zeffk
    Zeffk_dict = dict()

    Ycond = 1/np.linalg.cond(Y) #cond number sets the threshold for the sing values that make it through pinv
    Z = np.asmatrix(np.linalg.pinv(Y, rcond=Ycond*10))

    k = busidxs.index(busidx)
    if k == 0: #HERE assumes k==0 is infinite bus, dont want a Zeffk for the infinite bus (substation) and itself
        warning('Computing Zeff for the substation reference node')

    ei = np.asmatrix(np.zeros((len(Z),3)))
    e1 = np.asmatrix(np.zeros((len(Z),3)))
    ei[k*3:(k+1)*3,0:3] = np.eye(3)
    e1[0:3,0:3] = np.eye(3)
    Zeffk = np.transpose((e1 - ei))*Z*(e1 - ei)

    print('Zeffk for bus ' + str(bus) + ': ' + str(Zeffk))
    Zeffk_dict[bus] = Zeffk
    return Zeffk


def calc_Zeffks(Y, busidxs):
    Zeffk_dict = dict()

    Ycond = 1/np.linalg.cond(Y) #cond number sets the threshold for the sing values that make it through pinv
    Z = np.asmatrix(np.linalg.pinv(Y, rcond=Ycond*10))

    for k, bus in enumerate(busidxs):
        if k == 0: #HERE assumes k==0 is infinite bus, dont want a Zeffk for the infinite bus (substation) and itself
            continue

        ei = np.asmatrix(np.zeros((len(Z),3)))
        e1 = np.asmatrix(np.zeros((len(Z),3)))
        ei[k*3:(k+1)*3,0:3] = np.eye(3)
        e1[0:3,0:3] = np.eye(3)
        Zeffk = np.transpose((e1 - ei))*Z*(e1 - ei)

        print('Zeffk for bus ' + str(bus) + ': ' + str(Zeffk))
        Zeffk_dict[bus] = Zeffk
    return (Zeffk_dict)

# def export_Zeffks(Zeffk_dict, busidx, path, testcase, puFlag=1):
def export_Zeffks(Zeffk_dict, busidx, testcase, puFlag=1):
    # current_directory = path
    current_directory = os.getcwd()
    #HHERE could move os.path back and then to LPBC so you dont have to manually move the files..
    if puFlag:
        final_directory = os.path.join(current_directory, 'Zeffks/' + str(testcase) + '/PU')
        # final_directory = os.path.join(current_directory, 'Zeffks_pu')
    else:
        final_directory = os.path.join(current_directory, 'Zeffks/' + str(testcase) + '/notPU')
    if not os.path.exists(final_directory):
        os.makedirs(final_directory)
    os.chdir(final_directory)
    for k, bus in enumerate(busidx):
        if k == 0: #HERE assumes k==0 is infinite bus
            continue
        #save Zeffk in a csv
        Zeffk_df = pd.DataFrame(Zeffk_dict[bus])
        Zeffk_df.to_csv('Zeffk_bus' + str(bus) + '.csv') #save Zeffk
    os.chdir(current_directory) #change directory back
    return

def export_Ybus(Ybus_pu, Ybus, testcase): #, filepath):
    # current_directory = filepath
    # final_directory = os.path.join(current_directory, 'Ybus')
    # if not os.path.exists(final_directory):
    #     os.makedirs(final_directory)
    # os.chdir(final_directory)

    # os.chdir(filepath)

    current_directory = os.getcwd()
    final_directory = os.path.join(current_directory, 'YbusMatrices/' + str(testcase))
    if not os.path.exists(final_directory):
        os.makedirs(final_directory)
    os.chdir(final_directory)

    Ybus_pu_df = pd.DataFrame(Ybus_pu)
    Ybus_df = pd.DataFrame(Ybus)
    #then save results to a csv
    Ybus_pu_df.to_csv('Ybus_pu.csv')
    Ybus_df.to_csv('Ybus.csv')

    os.chdir(current_directory)
    return

def calc_Ybus(myfeeder):
    #buses in Y are ordered according to busdict() order, which is in order of the rows of the excel file
    #need to put the substation in the first entry
    busidx = []
    for Vsrc, bus in myfeeder.Vsrcdict.items():
        substation = Vsrc #this doest support networks with more than one substation
        print('substation at node ' + str(substation))
    busidx.append(substation)
    for key, bus in myfeeder.busdict.items():
        if key != substation:
            busidx.append(key)

    nbuses = len(busidx)
    Ypu = np.zeros((nbuses*3, nbuses*3), dtype=np.complex_)
    Y = np.zeros((nbuses*3, nbuses*3), dtype=np.complex_)

    for key, line in myfeeder.linedict.items():
        # print('line ' + str(line) + ' Ypu erroneous: ' + str(line.Ypu))
        # print('line ' + str(line) + ' Ypu: ' + str(np.reciprocal(line.Zpu)))
        a = busidx.index(line.from_node.buskey)
        b = busidx.index(line.to_node.buskey)
        Ypu[a*3:(a+1)*3, b*3:(b+1)*3] = -np.linalg.pinv(line.Zpu)
        Ypu[b*3:(b+1)*3, a*3:(a+1)*3] = -np.linalg.pinv(line.Zpu)
        Y[a*3:(a+1)*3, b*3:(b+1)*3] = -np.linalg.pinv(line.Z)
        Y[b*3:(b+1)*3, a*3:(a+1)*3] = -np.linalg.pinv(line.Z)
        #below is wrong
        # Ypu[a*3:(a+1)*3, b*3:(b+1)*3] = -np.reciprocal(line.Zpu, out=np.zeros_like(line.Zpu), where=line.Zpu!=0) #want the admittance of each line
        # Ypu[b*3:(b+1)*3, a*3:(a+1)*3] = -np.reciprocal(line.Zpu, out=np.zeros_like(line.Zpu), where=line.Zpu!=0)
        # Y[a*3:(a+1)*3, b*3:(b+1)*3] = -np.reciprocal(line.Z, out=np.zeros_like(line.Z), where=line.Z!=0) #want the admittance of each line
        # Y[b*3:(b+1)*3, a*3:(a+1)*3] = -np.reciprocal(line.Z, out=np.zeros_like(line.Z), where=line.Z!=0)

    for key, xfmr in myfeeder.transdict.items():
        a = busidx.index(xfmr.w0_node.buskey)
        b = busidx.index(xfmr.w1_node.buskey)
        Ypu[a*3:(a+1)*3, b*3:(b+1)*3] = -np.reciprocal(xfmr.Zpu, out=np.zeros_like(xfmr.Zpu), where=xfmr.Zpu!=0)
        Ypu[b*3:(b+1)*3, a*3:(a+1)*3] = -np.reciprocal(xfmr.Zpu, out=np.zeros_like(xfmr.Zpu), where=xfmr.Zpu!=0)
        Zbase = myfeeder.busdict[xfmr.w0_node.buskey].Zbase #the non-pu Z of a transformer is not defined, so I jsut used the Zbase at w0
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

# print(os.getcwd())
baseDirectory = os.getcwd()
os.chdir('../') #os.chdir() is eq to terminal cd
os.chdir('../') #second call brings you back to energise implementation
# print(os.getcwd())
SPBCfolderPath = os.getcwd() + '/SPBC/'
# print(SPBCfolderPath)
os.chdir(baseDirectory) #put directory back to LPBC folder
# print(os.getcwd())

'IEEE13_UNBALANCED'
# filepath = SPBCfolderPath + "IEEE13/"
# modelpath = filepath + "001 phasor08_IEEE13_OPAL.xls"
# loadfolder = SPBCfolderPath + "IEEE13/"
# loadpath = loadfolder + "001_phasor08_IEEE13_T12-3.xlsx"
# feederID = 'IEEE13'
# testcase = '13unb' #testcase needs to match the textcase in lpbcwrapper so that the LQR controller can find the right network model

'IEEE13_BALANCED'
filepath = SPBCfolderPath + "IEEE13_bal/"
modelpath = filepath + "016_GB_IEEE13_balance_reform.xlsx"
loadfolder = SPBCfolderPath + "IEEE13_bal/"
loadpath = loadfolder + "016_GB_IEEE13_balance_norm03.xlsx"
feederID = 'IEEE13'
testcase = '13bal'

# '33NF'
# filepath = SPBCfolderPath + "33/"
# modelpath = filepath + "005_GB_UCB33_opal_v3.xlsx"
# loadfolder = SPBCfolderPath + "33/"
# loadpath = loadfolder + "005_GB_UCB33_time_sigBuilder_Q_13_14_norm03.xlsx"
# feederID = 'UCB33'
# testcase = '33'

'PL0001'
# filepath = SPBCfolderPath + "PL0001/"
# modelpath = filepath + "PL0001_OPAL_working_reform_xfmr.xlsx"
# loadfolder = SPBCfolderPath + "PL0001/"
# loadpath = loadfolder + "PL0001_July_Q_F.xlsx"
# feederID = 'PL0001'
# testcase == 'PL0001'

# Specify substation kV, kVA bases, and the number of timesteps in the load data
if feederID == 'IEEE13':
    subkVbase_phg = 4.16/np.sqrt(3)
    subkVAbase = 5000.
if feederID == 'UCB33':
    subkVbase_phg = 12.47/np.sqrt(3)
    subkVAbase = 3000.
if feederID == 'PL0001':
    subkVbase_phg = 12.6/np.sqrt(3)
    subkVAbase = 1500.

# the SPBC (Jaspers main_run_3.py code) doesnt divide subkVAbase by 3?
modeldata = pd.ExcelFile(modelpath)
actpath = loadpath

################# Dummy Vars to make a Feeder ######################
timesteps = 1
timestepcur = 0

# #[HIL] - input constnats for PV forecasting
# PV_on = False # True for ON
# PVnodes = ['671','680']
#
# PVforecast = {}
# PVforecast['on_off'] = PV_on
# for node in PVnodes: # this sets all nodes the same, would have to manually change fields to have different inputs for different nodes
#     PVforecast[node] = {}
#     PVforecast[node]['on_off'] = PV_on
#     PVforecast[node]['lat'] = 37.87
#     PVforecast[node]['lon'] = -122
#     PVforecast[node]['maridian'] = -120
#     PVforecast[node]['PVfrac'] = 0.3
#
# refphasor = np.ones((3,2))
# refphasor[:,0]=1
# refphasor[:,1]=[0,4*np.pi/3,2*np.pi/3]
#
# Psat_nodes = []
# Qsat_nodes = []
###############################################################
#####################
# Create feeder object
myfeeder = feeder(modelpath,loadfolder,loadpath,actpath,timesteps,subkVbase_phg,subkVAbase)
#HERE feeder doestn actually put the shunt portion of the line impedances on the network (bdict doesnt go anywhere)
# myfeeder = feeder(modelpath,loadfolder,loadpath,actpath,timesteps,timestepcur, subkVbase_phg,subkVAbase,refphasor,Psat_nodes,Qsat_nodes,PVforecast)


# pprint(vars(myfeeder)) #pprint does help here

(Ypu, Y, busidx) = calc_Ybus(myfeeder)

# print('busidx : ' + str(busidx))

#need to put this in the same function
Zeffk_PU_dict = calc_Zeffks(Ypu, busidx)
Zeffk_notPU_dict = calc_Zeffks(Y, busidx)

# export_Zeffks(Zeffk_PU_dict, busidx, filepath, puFlag=1)
export_Zeffks(Zeffk_PU_dict, busidx, testcase, puFlag=1)
export_Zeffks(Zeffk_notPU_dict, busidx, testcase, puFlag=0)

# export_Ybus(Ypu, Y, filepath)
export_Ybus(Ypu, Y, testcase)
