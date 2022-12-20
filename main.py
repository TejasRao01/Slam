#!/usr/bin/env python

"""
Mobile robot simulation setup
@author: Bijo Sebastian 
"""

#Import libraries
import time

#Import files
import initialize_sim
import controller
import matplotlib.pyplot as plt
import sim
import localize

def main():
    global client_ID
    client_ID = initialize_sim.sim_init()
    if (client_ID > -1):

        #Start simulation
        if (initialize_sim.start_simulation(client_ID)):
            
            
            #Stop Robot
            open('obj_coordinates.txt', 'w').close()
            open('motor_ticks.text', 'w').close()
            
            goal = 0
            handles = initialize_sim.Handles(client_ID)
            initialize_sim.get_obj_coordinates(handles)
            ctrl = controller.Controller(client_ID, goal, handles)
            ctrl.set_vel(0,0)
            #fig, ax = plt.subplots(figsize=(10, 8))
            
            open('scan.txt', 'w').close()
            open('actual_pos.txt', 'w').close()
            
            while goal < 2:#len(handles.goal_handles) :
                [goal,currPos] = ctrl.move(client_ID,goal)
                time.sleep(0.1)
                
                res, signal  = sim.simxGetStringSignal(client_ID, 'c', sim.simx_opmode_blocking)
                lidar_data = sim.simxUnpackFloats(signal)
                
               
                l = localize.Localize(client_ID, 'c')
                
                while not l.get_signal():
                    time.sleep(0.01)
            ctrl.set_vel(0,0)    
                
            
        else:
            print ('Failed to start simulation')
    else:
        print ('Failed connecting to remote API server')
    
    #stop robots
    #initialize_sim.sim_shutdown(client_ID)
    time.sleep(2.0)
    return

#run
if __name__ == '__main__':

    main()                    
    print ('Program ended')
            

 
