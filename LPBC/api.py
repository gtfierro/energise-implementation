'''
list of request commands:
    
inverter:
    - f"http://131.243.41.47:9090/control?inv_id={inv},Batt_ctrl={batt},pf_ctrl={pf_ctrl}"

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


import requests
r = requests.get('http://131.243.41.118:9090/control?P_ctrl=0')
print(r)
