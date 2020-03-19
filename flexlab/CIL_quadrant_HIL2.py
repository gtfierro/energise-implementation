#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul 31 18:x:43 x19

@author: energise
"""
import numpy as np
import time
from pymodbus.client.sync import ModbusTcpClient as ModbusClient

def modbustoOpal_quadrant(Pcmd_kVA, Qcmd_kVA, Pact, Qact, act_idxs, client):
    c = 500 / 1 / 1000
    id = 3
    inv_1 = 101
    inv_2 = 102
    inv_3 = 103
    act_idxs_registers = []
    pq_changed = []

    Pcmd_VA = Pcmd_kVA * 1000
    Qcmd_VA = Qcmd_kVA * 1000
    Pcmd_VA = Pcmd_VA.tolist()
    Qcmd_VA = Qcmd_VA.tolist()
    Pcmd_kVA = Pcmd_kVA.tolist()
    Qcmd_kVA = Qcmd_kVA.tolist()

    # P,Q commands in W and VAR (not kilo)

    # P1, P2, P3 = 1, 3, 5
    # Q1, Q2, Q3 = 2, 4, 6

    P1, P2, P3 = abs(Pcmd_VA[0]), abs(Pcmd_VA[1]), abs(Pcmd_VA[2])
    Q1, Q2, Q3 = abs(Qcmd_VA[0]), abs(Qcmd_VA[1]), abs(Qcmd_VA[2])

    sign_vec = [1, 1,
                1, 1,
                1, 1]
    sign_base = 2 ** 5 * sign_vec[0] + 2 ** 4 * sign_vec[1] + 2 ** 3 * sign_vec[2] + 2 ** 2 * sign_vec[3] + 2 ** 1 * \
                sign_vec[4] + 2 ** 0 * sign_vec[5]
    # sign_list = (np.array(sign_vec)*np.array(sign_base)).tolist()

    # mtx = [0]*6
    mtx = [P1, Q1, P2, Q2, P3, Q3, sign_base]
    mtx_register = np.arange(1, 8).tolist()
    print(mtx)
    try:

        # write switch positions for config
        for i in range(len(mtx)):
            print(mtx_register[i], mtx[i])
            client.write_registers(int(mtx_register[i]), mtx[i], unit=id)
        print('sent CIL')

        # client.write_registers(mtx_register, mtx, unit=id)
        # print('sent')

    except Exception as e:
        print(e)
    '''
    for i,j in zip(range(len(act_idxs)), act_idxs): #checks to see if any sign changes occured from last command
        if np.sign(Pcmd_kVA[i]) != np.sign(Pact[i]) or np.sign(Qcmd_kVA[i]) != np.sign(Qact[i]):
            act_idxs_registers.append(j)
            pq_changed.append(i)
    if len(act_idxs_registers) > 0: #if any quadrant changes, execute modbus, else return.
        value = [0] * len(act_idxs_registers)
        inv_act_idxs_registers = act_idxs_registers.copy()
        #client = ModbusClient(IP, port=PORT)

        for i in range(len(act_idxs_registers)): # determines which inverters have quadrant change
            if inv_act_idxs_registers[i] == 1:
                inv_act_idxs_registers[i] = inv_1
            elif inv_act_idxs_registers[i] == 2:
                inv_act_idxs_registers[i] = inv_2
            elif inv_act_idxs_registers[i] == 3:
                inv_act_idxs_registers[i] = inv_3

        for i,j in zip(pq_changed, range(len(act_idxs_registers))): # determines exact quadrant for inverter
            if Pcmd_kVA[i] >= 0 and Qcmd_kVA[i] >= 0: #quadrant 1
                value[j] = 1
            if Pcmd_kVA[i] < 0 and Qcmd_kVA[i] >= 0: #quadrant 2
                value[j] = 2
            if Pcmd_kVA[i] < 0 and Qcmd_kVA[i] < 0: #quadrant 3
                value[j] = 3
            if Pcmd_kVA[i] >= 0 and Qcmd_kVA[i] < 0: #quadrant 4
                value[j] = 4
        try:

            for i in range(len(act_idxs_registers)): # write quadrant changes to modbus registers
                client.write_registers(inv_act_idxs_registers[i], value[i] , unit = id)
                print('Quadrant change for inv:', inv_act_idxs_registers[i], 'to quadrant', value[i] )

        except Exception as e:
            print(e)
        
        finally:
            client.close()
            print('closed client')

    else:
        return
'''

IP = '131.243.41.14'
PORT = 504
x = 50

client = ModbusClient(IP, port=PORT)

Pcmd_kVA = np.array([[x,x,x],[-x,-x,-x],[-x,-x,-x], [x,x,x], [0,0,0]]) #3 phase each array is a new iteration command
Qcmd_kVA = np.array([[x,x,x],[x,x,x], [-x,-x,-x], [-x,-x,-x], [0,0,0]])
Pact = np.array([[0,0,0],[x,x,x], [-x,-x,-x], [-x,-x,-x], [x,x,x]])
Qact = np.array([[0,0,0],[x,x,x], [x,x,x], [-x,-x,-x], [-x,-x,-x]])
act_idxs = np.array([1,2,3]) #phases
for i in range(len(Pcmd_kVA)):
    modbustoOpal_quadrant(Pcmd_kVA[i], Qcmd_kVA[i], Pact[i], Qact[i], act_idxs, client)
    print('wait 60 sec...')
    time.sleep(60)
