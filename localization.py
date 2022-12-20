# -*- coding: utf-8 -*-
"""
Created on Tue Dec 13 17:29:20 2022

@author: Tejas Rao
"""

import matplotlib.pyplot as plt
import math
import time
import numpy as np
import robot_params

# Lidar data contains several lidar scans
# Lidar scan --> [angle, distance]
def process_data(): 
    f = open('scan.txt')
    lines = f.readlines()
    lidar_data = []
    #print(lines)
    for l in lines:
        l = l.split(' ')
        lidar_scan = []
        i = 0
        #print(len(l))
        while i < len(l)-1:    
           
            r = float(l[i+1])
            if r < 0.012:
                r = 10
            lidar_scan.append([float(l[i]), r]) 
            i += 2         
        
        lidar_data.append(lidar_scan)
        
    return lidar_data 

def compute_derivative(lidar_data):
    
    # Obtaining the lidar data each scan
    # Obtain range and scan index
    RayIndices = []
    Ranges = []
    Derivatives = []
    for lidar_scan in lidar_data:
        ScanRayIndices = []
        ranges = []
        for i in range(len(lidar_scan)):
            ScanRayIndices.append(i)
            ranges.append(lidar_scan[i][1])
        RayIndices.append(ScanRayIndices)
        Ranges.append(ranges)
        
        # Computing Derivatives 
        derivative = []
        for i in range(1,len(lidar_scan)-1):
            l = lidar_scan[i-1][1]
            r = lidar_scan[i+1][1]
            d = (r- l)/2
            derivative.append(d)
            
        Derivatives.append(derivative)
        
    return [Derivatives, Ranges, RayIndices]

# Returns the detected cylinders for each iteration
def find_cylinders(lidar_data):
    
    [Derivatives, Ranges, RayIndices] = compute_derivative(lidar_data)
        
    threshold = 0.1
    Cylinders = []
    
    for d in range(len(Derivatives)):
        ranges = Ranges[d]
        derivative = Derivatives[d]
        cylinders = []
        start = True
        avg_indice = 0
        n_indices = 0
        avg_depth = 0
        for i in range(len(derivative)):
            cylinder_offset = 0.3   
            if derivative[i] < -threshold :
                start = True
                avg_indice = 0
                n_indices = 0
                avg_depth = 0
                n_indices = 0
                avg_depth= 0 
                avg_indice = 0
                start = True
            if start == True and derivative[i] > threshold and n_indices > 0:
                avg_indice  = avg_indice / n_indices
                avg_depth = avg_depth / n_indices + cylinder_offset
                if avg_depth> 0.2:
                    cylinders.append([avg_depth, avg_indice])
                
                #if avg_indice < 25 and d == 0:
                    #print(i, derivative[i-10:i+10])
                start = False
            if start == True:
                avg_indice += i
                n_indices += 1
                avg_depth += ranges[i+1]
        
        Cylinders.append(cylinders)
    return [Derivatives, Ranges, RayIndices,Cylinders]

def rayInd2angle(ind, num_indices):
    total_ind = num_indices
    angle = ind * math.pi / (total_ind-1) 
    return angle
    
    
def plot_lidar_data(Derivatives,Ranges, RayIndices, Cylinders):
    
    scan_id = 0
    cylinders = Cylinders[scan_id]
    cyl_ray = []
    cyl_dist = []
    for i in range(len(cylinders)):
        cyl_ray.append(cylinders[i][1])
        cyl_dist.append(cylinders[i][0])
        
    ranges = Ranges[scan_id]
    ray_indices = RayIndices[scan_id]
    
    plt.plot(ray_indices, ranges)
    plt.plot(ray_indices[1:-1], Derivatives[0])
    plt.scatter(cyl_ray, cyl_dist)
    #plt.pause(0.1)
    #plt.clf()

def transform_coordinates(robot_x, robot_y, robot_theta, X, Y):
    theta = robot_theta - math.pi/2
    x_tf = []
    y_tf = []
    for i in range(len(X)):
        
        x_dash = X[i]
        y_dash = Y[i]
        x = x_dash * math.cos(theta) - y_dash * math.sin(theta) 
        y = x_dash * math.sin(theta) + y_dash * math.cos(theta) 
        
        x = x + robot_x
        y = y + robot_y
        
        x_tf.append(x)
        y_tf.append(y)
        
    return [x_tf, y_tf]
        
def plot_scan_2D(Ranges, RayIndices, actual_pos, Cylinders, cyl_actual_coordinates):
    
    # Obtain actual cylinder coordinates
    cyl_actual_coordinatesX = cyl_actual_coordinates[0]
    cyl_actual_coordinatesY = cyl_actual_coordinates[1]
    Cyl_act = []
    #print(len(Ranges), len(actual_pos))
    for scan_id in range(len(Ranges)):
        
        robot_x = actual_pos[scan_id][0]
        robot_y = actual_pos[scan_id][1]
        robot_theta =  actual_pos[scan_id][2]
        
        #print([robot_x, robot_y])
        
        ranges = Ranges[scan_id]
        ray_indices = RayIndices[scan_id]
        num_indices = len(ray_indices)
        #
        #print(num_indices)
        ray_indices = [rayInd2angle(ind, num_indices) for ind in ray_indices]
        X = []
        Y = []
        for r in range(len(ranges)):
            x = ranges[r] * math.cos(ray_indices[r])
            y = ranges[r] * math.sin(ray_indices[r])
            
            X.append(x)
            Y.append(y)
        [X, Y] = transform_coordinates(robot_x, robot_y, robot_theta, X, Y)
        
        [wallL, wallR, wallU, wallD] = get_corresponding_points_on_wall(X, Y)
                                           
        plt.scatter(wallL[0], wallL[1],c = 'black')
        plt.scatter(wallU[0], wallU[1],c = 'black')
        plt.scatter(wallR[0], wallR[1],c = 'black')
        plt.scatter(wallD[0], wallD[1],c = 'black')
        
        
        plt.scatter(robot_x, robot_y, marker = "o")
        
        ## Adding arrow to show direction 
        
        arrow_startX = robot_x
        arrow_startY = robot_y
        arrow_length = 0.5
        arrow_endX = robot_x  + arrow_length * math.cos(robot_theta)
        arrow_endY = robot_y + arrow_length * math.sin(robot_theta)       
        
        
        #Detected Cylinders 
        
        cylinders = Cylinders[scan_id]
        
        
        cyl_angles = []
        for i in range(len(cylinders)):
            cyl_angles.append(rayInd2angle(cylinders[i][1], num_indices))
        
        
        cyl_robotX = []
        cyl_robotY = []
        for i in range(len(cyl_angles)):
            r = cylinders[i][0]
            #print(r)
            if r > 0.5:
                cyl_robotX.append(r * math.cos(cyl_angles[i]))
                cyl_robotY.append(r * math.sin(cyl_angles[i]))   
        
        
        
        [cyl_worldX, cyl_worldY] = transform_coordinates(robot_x, robot_y, robot_theta, cyl_robotX, cyl_robotY)
        
        cyl_act = []
    
        for i in range(len(cyl_worldX)):
            cyl_act.append([cyl_worldX[i],cyl_worldY[i]])
        
        
        plt.scatter(cyl_actual_coordinatesX, cyl_actual_coordinatesY, marker = 'o', linewidths=10)
        plt.plot([arrow_startX, arrow_endX], [arrow_startY, arrow_endY])
        plt.scatter(cyl_worldX, cyl_worldY, marker = 'x')
        
        plt.scatter(X, Y)
        plt.xlim(-3, 8)
        plt.ylim(-8,3)
        plt.pause(0.5)
        plt.clf()
        
        
        Cyl_act.append(cyl_act)
    return Cyl_act


    
    for i in range(len(actual_pos)):
        
        theta = actual_pos[i][2]
    
    return theta

def get_pos():
    
    f = open('actual_pos.txt', 'r')
    lines = f.readlines()
    actual_pos = []
    for line in lines:
        #print(line)
        l = list(map(float, line.split(' ')))
        actual_pos.append(l)
        
        
    return actual_pos
    
def obtain_correspondeces(actual_pos, Cylinders, world_cylinders, num_indices):
    
    Cylinder_pairs = []
    for scan_id in range(len(Ranges)):   
        
        cylinder_pairs = []
        robot_x = actual_pos[scan_id][0]
        robot_y = actual_pos[scan_id][1]
        robot_theta =  actual_pos[scan_id][2]
        
        cylinders = Cylinders[scan_id]
        
        
        cyl_angles = []
        for i in range(len(cylinders)):
            cyl_angles.append(rayInd2angle(cylinders[i][1], num_indices))
        
        
        cyl_robotX = []
        cyl_robotY = []
        for i in range(len(cyl_angles)):
            r = cylinders[i][0]
            #print(r)
            if r > 0.5:
                cyl_robotX.append(r * math.cos(cyl_angles[i]))
                cyl_robotY.append(r * math.sin(cyl_angles[i]))   
        
        [cyl_worldX, cyl_worldY] = transform_coordinates(robot_x, robot_y, robot_theta, cyl_robotX, cyl_robotY)
        
        reference_cylinders = world_cylinders[scan_id]
        
        for i in range(len(cyl_worldX)):
            min_dist= max_radius
            closest_cyl = []
            for j in range(len(reference_cylinders)):
                
                dist = math.sqrt( (cyl_worldX[i] - reference_cylinders[j][0])**2 + (cyl_worldY[i] - reference_cylinders[j][1])**2     )
                
                if dist < min_dist:
                    min_dist = dist
                    closest_cyl = j
                    
            if not closest_cyl == []:
                cylinder_pairs.append(i,closest_cyl)
        Cylinder_pairs.append(cylinder_pairs)
    return Cylinder_pairs
                
                

        
            
def find_cylinder_pairs(Cylinders, Reference_cylinders, max_radius):
    
    Cylinder_pairs = []
    #for scan_id in range(len(Cylinders)):        
    for scan_id in range(1):
        cylinder_pairs = []
        cylinders = Cylinders[scan_id]
        reference_cylinders = Reference_cylinders
        # Make a loop over all cylinders and reference_cylinders.
        # In the loop, if cylinders[i] is closest to reference_cylinders[j],
        # and their distance is below max_radius, then add the
        # tuple (i,j) to cylinder_pairs, i.e., cylinder_pairs.append( (i,j) ).
        #print(cylinders)
        #print(reference_cylinders)  
        
        for i in range(len(cylinders)):
            min_dist = max_radius
            closest_cyl = []
            for j in range(len(reference_cylinders)):
                
                #print(cylinders, reference_cylinders)
                
                dist = distance(cylinders[i], reference_cylinders[j])
                print(dist)
                if  dist < min_dist:
                    min_dist = dist
                    closest_cyl = j
            
            if not closest_cyl == []:
                cylinder_pairs.append((i,closest_cyl))
        print('cylinder pairs',cylinder_pairs)
        Cylinder_pairs.append(cylinder_pairs)
    return Cylinder_pairs
        
def distance(p0, p1):
    return math.sqrt((p0[0] - p1[0])**2 + (p0[1] - p1[1])**2)   
    

# Given a point list, return the center of mass.
def compute_center(point_list):
    # Safeguard against empty list.
    if not point_list:
        return (0.0, 0.0)
    # If not empty, sum up and divide.
    sx = sum([p[0] for p in point_list])
    sy = sum([p[1] for p in point_list])
    return (float(sx) / len(point_list), float(sy) / len(point_list))

# Given a left_list of points and a right_list of points, compute
# the parameters of a similarity transform: scale, rotation, translation.
# If fix_scale is True, use the fixed scale of 1.0.
# The returned value is a tuple of:
# (scale, cos(angle), sin(angle), x_translation, y_translation)
# i.e., the rotation angle is not given in radians, but rather in terms
# of the cosine and sine.
def estimate_transform(left_list, right_list, fix_scale = False):
    
    if len(left_list) < 2 or len(right_list) < 2:
        return None

    # Compute left and right center.
    lc = compute_center(left_list)
    rc = compute_center(right_list)

    l_i = [tuple(np.subtract(l,lc)) for l in left_list]
    r_i = [tuple(np.subtract(r,rc)) for r in right_list]

    # print l_i
    # print r_i

    cs,ss,rr,ll = 0.0,0.0,0.0,0.0

    for i in range(len(left_list)):
        cs += (r_i[i][0] * l_i[i][0]) + (r_i[i][1] * l_i[i][1])
        ss += -(r_i[i][0] * l_i[i][1]) + (r_i[i][1] * l_i[i][0])
        #rr += (right_list[i][0] * right_list[i][0]) + (right_list[i][1] * right_list[i][1])
        #ll += (left_list[i][0] * left_list[i][0]) + (left_list[i][1] * left_list[i][1])
        rr += (r_i[i][0] * r_i[i][0]) + (r_i[i][1] * r_i[i][1])
        ll += (l_i[i][0] * l_i[i][0]) + (l_i[i][1] * l_i[i][1])


    #print cs, ss, rr, ll

    if rr == 0.0 or ll == 0.0:
        return None

    if fix_scale:
        la = 1.0
    else:
        la = math.sqrt(rr/ll)

    if cs == 0.0 or ss == 0.0:
        #c = 0.0
        #s = 0.0
        return None
    else:
        c = cs / math.sqrt((cs*cs) + (ss*ss))
        s = ss / math.sqrt((cs*cs) + (ss*ss))

    tx = rc[0] - (la * ((c * lc[0]) - (s * lc[1])))
    ty = rc[1] - (la * ((s * lc[0]) + (c * lc[1])))


    # --->>> Insert here your code to compute lambda, c, s and tx, ty.

    return la, c, s, tx, ty

# Given a similarity transformation:
# trafo = (scale, cos(angle), sin(angle), x_translation, y_translation)
# and a point p = (x, y), return the transformed point.
def apply_transform(trafo, p):
    la, c, s, tx, ty = trafo
    lac = la * c
    las = la * s
    x = lac * p[0] - las * p[1] + tx
    y = las * p[0] + lac * p[1] + ty
    return (x, y)

def correct_pose(pose, trafo):
    la, c, s, tx, ty = trafo
    #print 'trafo:', trafo
    old_x = pose[0]
    old_y = pose[1]
    old_theta = pose[2]
    #print 'pose: ', pose
    x, y = apply_transform( trafo, (old_x,old_y) )
    theta = old_theta + np.atan2(s,c)
    #print 'new pose: ', (x,y,theta)
    return (x, y, theta)  # Replace this by the corrected pose.
    #return(old_x,old_y,theta)


def delta_pos(x,y, theta, motor_ticks_curr, motor_ticks_old):
    
    deltaL = robot_params.pioneer_wheel_radius * (motor_ticks_curr[0] - motor_ticks_old[0])
    deltaR = robot_params.pioneer_wheel_radius * (motor_ticks_curr[1] - motor_ticks_old[1])
    
    Xnew = x + (deltaL + deltaR)/ (2) * math.cos(theta + (deltaR - deltaL)/ (2 * robot_params.pioneer_track_width))
    Ynew = y + (deltaL + deltaR)/ (2) * math.sin(theta + (deltaR - deltaL)/ (2 * robot_params.pioneer_track_width))
    Thetanew = (deltaR - deltaL)/ (2)
    
    return [Xnew, Ynew, Thetanew]


def pred_pos(initial_pos):
    f = open('motor_ticks.txt', 'r')
    
    lines = f.readlines()
    #print(len(lines))
    motor_ticks = []
    for l in lines:
        motor_ticks.append(list(map(float, l.split())))
    
    
    
    for i in range(len(motor_ticks)):
        motor_leftx = motor_ticks[i][0]
        motor_lefty = motor_ticks[i][1]
        motor_rightx = motor_ticks[i][2]
        motor_righty = motor_ticks[i][3]
    
        x = (motor_leftx + motor_rightx) / 2 
        y = (motor_righty + motor_lefty) / 2
        
        angle = np.arctan2((motor_righty - motor_lefty), (motor_rightx - motor_leftx))
        theta = angle + math.pi / 2 
        
        plt.scatter(x,y)
        plt.plot((x, x + 0.2 * math.cos(theta)), (y, y + 0.2 * math.sin(theta)))
        plt.pause(0.2)
        plt.clf()
        plt.xlim(-3, 8)
        plt.ylim(-8,3)
        
    
    
    
    
def get_corresponding_points_on_wall(X, Y,
                                     arena_left = -2.5, arena_right = 7.5,
                                     arena_bottom = -7.5, arena_top = 2.5,
                                     eps = 0.2):
    
    wallLx = []
    wallRx = []
    wallUx = []
    wallDx = []
    
    wallLy = []
    wallRy = []
    wallUy = []
    wallDy = []
    
    
    for i in range(len(X)):
        x = X[i]
        y = Y[i]
        
        if abs(x - arena_left) < eps:
            wallLx.append(arena_left)
            wallLy.append(y)
            
        elif abs(x - arena_right) < eps:
            wallRx.append(arena_right)
            wallRy.append(y)
            
        elif abs(y- arena_bottom) < eps:
            wallDx.append(x)
            wallDy.append(arena_bottom)
            
        elif abs(y - arena_top) < eps:
            wallUx.append(x)
            wallUy.append(arena_top)
    
    wallL = [wallLx, wallLy]
    wallR = [wallRx, wallRy]
    wallU = [wallUx, wallUy]
    wallD = [wallDx, wallDy]
    
    return [wallL, wallR, wallU, wallD]


def plot_pred_pos(pred_pos, actual_pos):
    
    X = []
    Y = []
    
    X1 = []
    Y1 = []
    for i in range(len(pred_pos)):
        
        x = pred_pos[i][0]
        y = pred_pos[i][1]
        
        x1 = actual_pos[i][0]
        y1 = actual_pos[i][1]
        
        X.append(x)
        Y.append(y)
        
        X1.append(x1)
        Y1.append(y1)
    
    #print(X, Y)
        plt.scatter(X, Y)
        plt.scatter(X1, Y1, c = 'red')
        plt.pause(0.05)
        plt.clf()
        plt.xlim(-3, 8)
        plt.ylim(-8,3)
        
    return
    

if __name__ == '__main__':

    actual_pos = get_pos() 
    predicted_pos = pred_pos(actual_pos[0])
# =============================================================================
#     for i in range(len(actual_pos)):
#         print(actual_pos[i][1], predicted_pos[i][1])
#     
#     lidar_data = process_data()   
#     [Derivatives, Ranges, RayIndices,Cylinders] = find_cylinders(lidar_data)
#     [] 
#     
# =============================================================================
# =============================================================================
#     
#     time.sleep(2)
#     cyl_actual_coordinatesX = [2.2497, 3.5252, 6.2249, 6.7998, 6.8998, 3.0504, 0.2245, -0.6747]
#     cyl_actual_coordinatesY = [1.7498, -0.8 , 0.9749, -1.75, -5.2249, -3.800, -6.3251, -2.7]
#     cyl_actual_coordinates = [cyl_actual_coordinatesX,cyl_actual_coordinatesY]
#     
#     Cyl_act = plot_scan_2D(Ranges, RayIndices, actual_pos, Cylinders,cyl_actual_coordinates)
#     
#     
#     
#     
#     
#     Reference_cylinders = [[cyl_actual_coordinatesX[i],cyl_actual_coordinatesY[i]] for i in range(len(cyl_actual_coordinatesX))]
#     max_radius = 3
#     cylinder_pairs = find_cylinder_pairs(Cyl_act, Reference_cylinders, max_radius)
#     
# =============================================================================
    
   # plot_pred_pos(predicted_pos,actual_pos)
    
    
