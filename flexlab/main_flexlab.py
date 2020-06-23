#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul 31 10:03:22 2019

@author: jasperpakshong
"""

'''

Main for running simulations - used to:
    # set ORT config & act scales by reading in excel sheet from sw_mat folder
    # initialize run SPBC?
    # initialize run LPBC?
    # send commands to load racks?
    # ...

'''
# In[imports]:

# import...
import pandas as pd
import numpy as np

from ORT_modbus_energise import *
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
# filename = 'HIL_switch_matrix_13NF_unbal_CIL.xlsx'
filename = 'HIL_switch_matrix_33NF_bal_CIL.xlsx'
# filename = 'HIL_switch_matrix_PL0001_CIL.xlsx'

# filename = 'HIL2_switch_matrix_13NF_bal_offset.xlsx'
# filename = 'HIL_switch_matrix_13NF_unbal_offset.xlsx'
# filename = 'HIL_switch_matrix_33NF_bal_offset.xlsx'
# filename = 'HIL_switch_matrix_PL0001_offset_PO.xlsx'
filepath = pathname+filename

test_ID = 'T8.1'
sim_length_min = 50 # amount of time to record for in minutes
# sim_length_min = 1/60
# In[run]:

# run functions...

dfsw_in,dfsw_out = read_sw_mat(filepath)

#set_switches(dfsw_in,dfsw_out,test_ID)


sim_start_stop(dfsw_in,dfsw_out,test_ID,sim_length_min)
# sim_start_pause_stop(1)


# sim_stop()
