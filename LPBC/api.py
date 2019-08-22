'''
list of request commands:
    
inverter:
    - f"http://131.243.41.47:9090/control?Batt_ctrl={batt},pf_ctrl={pf_ctrl},inv_id={inv}"

load racks:
    http://131.243.41.118:9090/control_enable
        -- enable
    http://131.243.41.118:9090/control?P_ctrl=1000 
        -- set all
    http://131.243.41.118:9090/control?group_id=1,P_ctrl=1000 
        -- set one
    http://131.243.41.118:9090/control_disable
        -- disable, however we are now just setting all to 0 when done
        
'''

# load rack values:
P_ctrl = 0
group_id = 0 # [0,1,2]

# inverter values:
Batt_ctrl = 0
pf_ctrl = 1
inv_id = 2 # [1,2,3]

import requests

# load racks:
#r = requests.get('http://131.243.41.118:9090/control_enable’)
#r = requests.get('http://131.243.41.118:9090/control_disable’)

#r = requests.get(f'http://131.243.41.118:9090/control?P_ctrl={P_ctrl}’)
#r = requests.get(f'http://131.243.41.118:9090/control?P_ctrl={P_ctrl},group_id={group_id}’)

# inverter
#r = requests.get(f'http://131.243.41.47:9090/control?Batt_ctrl={Batt_ctrl}')  # works
#r = requests.get(f'http://131.243.41.47:9090/control?Batt_ctrl={Batt_ctrl},inv_id={inv_id}')  # works
#r = requests.get(f'http://131.243.41.47:9090/control?Batt_ctrl={Batt_ctrl},pf_ctrl={pf_ctrl}')  # ?
#r = requests.get(f'http://131.243.41.47:9090/control?Batt_ctrl={Batt_ctrl},pf_ctrl={pf_ctrl},inv_id={inv_id}')  # ?

print(r)