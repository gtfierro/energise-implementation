
import asyncio

from KM_Zestwrapper_CILfunc import *

import numpy as np
from pymodbus.client.sync import ModbusTcpClient as ModbusClient

import pandas as pd
from ORT_modbus_energise import *

def CIL_debug():
    IP = '131.243.41.14'
    PORT = 504
    id = 3

    #IP = '131.243.41.14'
    #PORT = 502
    #id = 1

    # Connect to client
    client = ModbusClient(IP, port=PORT)
    '''
    try:
        client.connect()
        client.write_registers(int(1), int(50000), unit=id)
        print('sent')

    except Exception as e:
        print(e)

    finally:
        client.close()
    '''


    #P,Q commands in W and VAR (not kilo)

    #c = 500/3.3/1000

    #P1, P2, P3 = 1, 3, 5
    #Q1, Q2, Q3 = 2, 4, 6

    #RESET REGISTERS BACK TO ZERO TO RESTART OR END CIL TESTING
      # scaling ratio
    P1, P2, P3 = 0,0,0
    Q1, Q2, Q3 = 0,0,0
    # P1, P2, P3 = 16666, 16666, 16666  #VA command corresponding to .1 PU for 13bal (with Sratio division included)
    # Q1, Q2, Q3 = 16666, 16666, 16666


    # set signs of commands through sign_vec
    #           P,Q      1 is positive, 0 is negative
    sign_vec = [0,0,
                0,0,
                0,0]
    sign_base = 2**5 * sign_vec[0] + 2**4 * sign_vec[1] + 2**3 * sign_vec[2] + 2**2 * sign_vec[3] + 2**1 * sign_vec[4] + 2**0  * sign_vec[5]
    # sign_list = (np.array(sign_vec)*np.array(sign_base)).tolist()

    #mtx = [0]*6
    mtx = [P1,Q1,P2,Q2,P3,Q3,sign_base]
    mtx_register = np.arange(1,8).tolist()


    try:
        client.connect()
        # write switch positions for config
        for i in range(len(mtx)):
            client.write_registers(int(mtx_register[i]), mtx[i], unit=id)
        print('sent')

        #client.write_registers(mtx_register, mtx, unit=id)
        #print('sent')


    except Exception as e:
        print(e)

    finally:
        client.close()



def main_flexlab_CIL():
    '''
    functions from ORT_modbus_energise:
        ~ set up to copy and paste functions into command line of python console
        ~ need to have run read_sw_mat before sim_start_stop

        dfsw_in,dfsw_out = read_sw_mat(filepath)
            ~ read in the sw matrix excel file that has switch configs for each test case on that feeder
            ~ define relative folder in pathname and filename in filename

        sim_start_stop(dfsw_in,dfsw_out,test_ID,sim_length_min)
            ~ sends switch values of associated test_ID [i.e. 'T1.1']
            ~ along with a simulation flag signal of 1 to begin recording data
            ~ at end of sim_time_min [minutes], sends simulation flag of 0 to stop recording data
            ~ need interpolation

        sim_start()
            ~ sends simulation flag signal of 1
            ~ starts simulation monitoring and load reading
        sim_stop()
            ~ sends simulation flag signal of 0
            ~ stops monitoring, resets load to timestep 0 and stops load reading
        sim_pause()
            ~ sends simulation flag signal of 3
            ~ this should pause load at timestep 1
            ~ first column should be deleted or ignored


        (sending a 2 should pause load reading, if one is passed afterwards, it will continue from this point)


        datetime function to get datetime from epoch....
            import datetime as dt
            dt.datetime.fromtimestamp(1565048195.416524)

    '''

    # In[settings]:

    pathname = 'sw_mat_HIL2/'
    # filename = 'HIL2_switch_matrix_13NF_bal.xlsx'
    filename = 'HIL2_switch_matrix_13NF_bal_CILdebug.xlsx'
    # filename = 'HIL_switch_matrix_13NF_unbal_CIL.xlsx'
    # filename = 'HIL_switch_matrix_33NF_bal_CIL.xlsx'
    # filename = 'HIL_switch_matrix_PL0001_CIL.xlsx'
    filepath = pathname+filename

    test_ID = 'T3.3' # which test case you want to run
    # test_ID = 'T8.1'
    # test_ID = 'T9.3'
    sim_length_min = 50 # amount of time to record for in minutes
    # sim_length_min = 1/60
    # sim_length_min = 60
    # In[run]:

    # run functions...

    dfsw_in,dfsw_out = read_sw_mat(filepath)

    #set_switches(dfsw_in,dfsw_out,test_ID)

    # sim_start_stop(dfsw_in,dfsw_out,test_ID,sim_length_min)
    sim_start_keith(dfsw_in,dfsw_out,test_ID,sim_length_min)

    # sim_start_pause_stop(sim_length_min)





# caution = 2
# stepLength = 10 #this is rate in Zestwrapper in seconds
HistLength = 4
# Zeffk_init_mults = [.5, .75, 1.25, 1.5, 2]
Zeffk_init_mults = [.75, 1.25]
# Zeffk_init_mults = [.75]

# print('@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ DONE SLEEPING')
# asyncio.sleep(caution)
# print('@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ DONE SLEEPING')

for i in np.arange(len(Zeffk_init_mults)):
    CIL_debug() #initialize the HIL
    # time.sleep(5)
    main_flexlab_CIL() #Run the HIL
    # time.sleep(5)

    loop = asyncio.get_event_loop()
    buildZestimators(loop, i, Zeffk_init_mults[i],  HistLength)

    print('got loop, running DEGC process')
    loop.run_forever()
    # loop.stop() is implemented inside the LPBC script once it has successfully implemented enough iterations
    loop.close() #close DEGC process

    sim_stop() #stop HIL run
