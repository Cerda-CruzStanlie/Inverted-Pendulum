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
    data = data[0] / 255 + data[1]/64/255
    data = data
    return data

def normalize(curr_position,rest_position): #Normalize to rest position
    current_angle = read_raw_angle()
    normd = curr_position - rest_position
    return normd

################## ODRIVE ################
mass = 0.5  # Kg weights = 0.090
length = 0.11  # Meters
I = mass * length * 9.8

async def controller(odrive):
    await asyncio.sleep(1)
    #Run for set time delay example runs for 15 seconds.
    odrive.set_controller_mode("torque_control")
    stop_at = datetime.now() + timedelta(seconds=10000)
    time = 0
    ## Initilize Encoder ##
    rest_pos = read_raw_angle()
    val = 0
    hturns = 0
    odrive.set_torque(1)
    sum_e = 0
    err_last = 0
    while datetime.now() < stop_at:
		#### Encoder ######
        prev_val = val
        val = read_raw_angle()
        diff = val - prev_val
        if diff < -.5:  # This is to wrap the position, it is based on the fact that you have a 1:2 gear ratio. I would change this to 1:1
            hturns += 1
        elif diff> 0.5:
            hturns -= 1
        position = val + hturns
        position = normalize(position,rest_pos)
        err = position*np.pi
        
        #### Gains ######
        K1 = 3
        K2 = 0.000001
        K3 = 2
        
        sum_e += err
        del_e = err-err_last
        # Calculate next wheel position
        time -= asyncio.get_event_loop().time()
        next_vel = (-err*K1 - K2*sum_e - del_e/time*K3)
        time = asyncio.get_event_loop().time()
        odrive.set_torque(next_vel)
        
        err_last = err
        # Limit next_torque to between -0.129 and 0.129
        # next_torque = max(-1, min(1, next_torque))
        
 
        print(f"Arm Pos: {round(position,3)} (rad),\t called vel:{round(next_vel,3)} (Rad/s),\t set vel: {odrive.velocity}")

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
        
