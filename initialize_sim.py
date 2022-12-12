# -*- coding: utf-8 -*-
"""
Created on Mon Dec 12 18:12:02 2022

@author: Tejas Rao
"""

import sim 
import sim_params

def sim_init():

  global client_ID
  
  #Initialize sim interface
  sim.simxFinish(-1) # just in case, close all opened connections
  client_ID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim    
  if client_ID!=-1:
    print ('Connected to remote API server')
    return client_ID
  else:
    return -1

def start_simulation(client_ID):

  ###Start the Simulation: Keep printing out status messages!!!
  res = sim.simxStartSimulation(client_ID, sim.simx_opmode_oneshot_wait)

  if res == sim.simx_return_ok:
    print ("---!!! Started Simulation !!! ---")
    return True
  else:
    return False



class Handles():
    
    def __init__(self, client_ID):
        
        self.client_ID = client_ID
        self.get_handles()
        

    def get_handles(self):
    
        left_motor_name = '/Pioneer1/leftMotor'
        right_motor_name = '/Pioneer1/rightMotor'
        goal_names = ['goal_1','goal_2','goal_3','goal_4','goal_5','goal_6','goal_7','goal_8']
        pioneer_name = '/Pioneer1'
        lidar_sensor_name = '/Pioneer1/Lidar2D/sensor'
        lidar_joint_name = '/Pioneer1/Lidar2D/joint'
        
        # Motor Handles
        client_ID = self.client_ID
        
        # Motor Handles 
        res, self.left_motor_handle = sim.simxGetObjectHandle(client_ID, left_motor_name , sim.simx_opmode_blocking)
        res, self.right_motor_handle = sim.simxGetObjectHandle(client_ID, right_motor_name , sim.simx_opmode_blocking)
        
        print(self.left_motor_handle, self.right_motor_handle)
        #Goal Handles 
        self.goal_handles = []
        for i in range(len(goal_names)):
            res, goal_handle = sim.simxGetObjectHandle(client_ID, goal_names[i], sim.simx_opmode_blocking)
            self.goal_handles.append(goal_handle)
        
        res,self.pioneer_handle = sim.simxGetObjectHandle(client_ID, pioneer_name, sim.simx_opmode_blocking)
        
        #Lidar Handles
        
        res, self.lidar_sensor_handle = sim.simxGetObjectHandle(client_ID,lidar_sensor_name , sim.simx_opmode_blocking)
        res, self.lidar_joint_handle = sim.simxGetObjectHandle(client_ID, lidar_joint_name, sim.simx_opmode_blocking)
    
    
    

def sim_shutdown(client_ID):
  #Gracefully shutdown simulation


  #Stop simulation
  res = sim.simxStopSimulation(client_ID, sim.simx_opmode_oneshot_wait)
  if res == sim.simx_return_ok:
    print ("---!!! Stopped Simulation !!! ---")

  # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
  sim.simxGetPingTime(client_ID)

  # Now close the connection to CoppeliaSim:
  sim.simxFinish(client_ID)      

  return            

