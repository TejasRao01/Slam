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


def main():
    global client_ID
    client_ID = initialize_sim.sim_init()
    if (client_ID > -1):

        #Start simulation
        if (initialize_sim.start_simulation(client_ID)):
            
            #Obtain sim_params
            #sim_params = get_sim_params()
            
            #Stop Robot
            
            goal = 0
            handles = initialize_sim.Handles(client_ID)
            ctrl = controller.Controller(client_ID, goal, handles)
            ctrl.set_vel(0,0)
            fig, ax = plt.subplots(figsize=(10, 8))
            
            
            open('actual_pos.txt', 'w').close()
            while goal < len(handles.goal_handles) :
                [goal,currPos] = ctrl.move(client_ID,goal)
                time.sleep(0.1)
                
                
                
            
            ctrl.set_vel(0,0)    
                
              
            
# =============================================================================
#             #Stop robot
#             set_vel(0, 0)
#             set_vel(2, 0)
#             time.sleep(10)
#             
#             set_vel(0,0)
# =============================================================================

# =============================================================================
#             #Obtain goal state
#             goal_state = sim_interface.get_goal_pose()
# 
#             #Obtain robots position
#             robot_state = sim_interface.localize_robot()
# 
#             while not control.at_goal(robot_state, goal_state):
#                 [V,W] = control.gtg(robot_state, goal_state)
#                 sim_interface.setvel_pioneers(V, W)
#                 time.sleep(0.5)
#                 robot_state = sim_interface.localize_robot()
#                 goal_state = sim_interface.get_goal_pose()
#                                 
#             #Stop robot
#             sim_interface.setvel_pioneers(0.0, 0.0)
# =============================================================================

        else:
            print ('Failed to start simulation')
    else:
        print ('Failed connecting to remote API server')
    
    #stop robots
    initialize_sim.sim_shutdown(client_ID)
    time.sleep(2.0)
    return

#run
if __name__ == '__main__':

    main()                    
    print ('Program ended')
            

 