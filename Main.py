import pyodrivecan
import asyncio
import math
import numpy as np
from datetime import datetime, timedelta
import smbus
import time
import os


################## ENCODER ###############
def read_raw_angle(): # Function to read raw angle from the encoder
    data = smbus.SMBus(1).read_i2c_block_data(0x40, 0xFE, 2)
    return data[0] / 255 + data[1]/64/255

################## ODRIVE ################

async def controller(odrive):
    await asyncio.sleep(1)
    #Run for set time delay example runs for 15 seconds.
    odrive.set_controller_mode("torque_control")
    stop_at = datetime.now() + timedelta(seconds=10000)
    dt = 0
    ## Initilize Encoder ##
    rest_pos = read_raw_angle()
    val = 0
    hturns = 0
    odrive.set_torque(1)
    sum_e = 0
    err_last = 0

    #### Gains ######
    K1 = 3
    K2 = 0.1
    K3 = 0.5
    while datetime.now() < stop_at:
		# ### Encoder ######
        prev_val = val
        val = read_raw_angle()
        diff = val - prev_val
        if diff < -.5:  # This is to wrap the position, it is based on the fact that you have a magnetic encoder that goes from 0-1
            hturns += 1
        elif diff> 0.5:
            hturns -= 1
        position = val + hturns
        e = position - rest_pos
        
        sum_e += e
        de = e-err_last
        # Calculate next wheel input
        dt -= asyncio.get_event_loop().time()
        u = (-e*K1 - K2*sum_e - de/dt*K3)
        odrive.set_torque(u)
        dt = asyncio.get_event_loop().time()
        err_last = e
        

        await asyncio.sleep(0.0005)  # 15ms sleep, adjust based on your control loop requirements


#Set up Node_ID 10 ACTIV NODE ID = 10
odrive = pyodrivecan.ODriveCAN(0)

# Run multiple busses.
async def main():
    odrive.clear_errors(identify=False)
    print("Cleared Errors")
    await asyncio.sleep(1)

    #Initalize odrive
    odrive.initCanBus()

    
    print("Put Arm at bottom center to calibrate Zero Position.")
    await asyncio.sleep(1)
    cur_pos = odrive.position
    await asyncio.sleep(1)
    print(f"Encoder Absolute Position Set: {cur_pos}")

    #odrive.setAxisState("closed_loop_control")
    odrive.setAxisState("open_loop_control")

    #add each odrive to the async loop so they will run.
    await asyncio.gather(
        odrive.loop(),
        controller(odrive) 
    )



################ RUN STUFF #################
if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("KeyboardInterrupt caught, stopping...")
        odrive.estop()
        
