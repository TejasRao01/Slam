# -*- coding: utf-8 -*-
"""
Created on Tue Dec 13 17:29:20 2022

@author: Tejas Rao
"""




import matplotlib.pyplot as plt
import math
import time

def plotCoord():
    f = open('scan.txt')
    l = f.readline()
    print(l.split())
    l = l.split(' ')
    lidar_data = []
    i = 0
    while i < len(l)-1:    
        #print(i, [(l[i][0]), (l[i][1])])
        lidar_data.append([float(l[i]), float(l[i+1])])
        i += 2 
        
    lidar_data.reverse()    
    print(lidar_data)
    X = []
    Y = []
    for i in range(len(lidar_data)):
        r = lidar_data[i][1]
        if r == 0:
            continue
        theta = lidar_data[i][0]
        
        x = r * math.sin(theta)
        y = r * math.cos(theta)
        
        X.append(x)
        Y.append(y)
    plt.figure() 
    plt.scatter(X,Y)

def plotVsRay():
    f = open('scan.txt')
    l = f.readline()
    print(l.split())
    l = l.split(' ')
    lidar_data = []
    i = 0
    while i < len(l)-1:    
        #print(i, [(l[i][0]), (l[i][1])])
        lidar_data.append([float(l[i]), float(l[i+1])])
        i += 2 
        
    lidar_data.reverse()
    print(lidar_data)
    X = []
    Y = []
    for i in range(len(lidar_data)):
        X.append(i)
        Y.append(lidar_data[i][1])
    
    plt.plot(X,Y)

def process_data():
    f = open('scan.txt')
    l = f.readline()
    print(l.split())
    l = l.split(' ')
    lidar_data = []
    i = 0
    while i < len(l)-1:    
        #print(i, [(l[i][0]), (l[i][1])])
        r = float(l[i+1])
        if r < 2:
            r = 10
        lidar_data.append([float(l[i]), r])
        i += 2 
    
    lidar_data.reverse()
    return lidar_data
    
def compute_derivative(lidar_data):
    X = []
    Y = []
    for i in range(len(lidar_data)):
        X.append(i)
        Y.append(lidar_data[i][1])
    
    plt.plot(X,Y)

    # Computing Derivatives 
    derivative = []
    for i in range(1,len(lidar_data)-1):
        l = lidar_data[i-1][1]
        r = lidar_data[i+1][1]
        
        
        #if l > min_dist and r > min_dist:
        d = (r- l)/2
        derivative.append(d)
    plt.plot(X[1:-1],Y[1:-1])
    plt.plot(X[1:-1], derivative)
    #plt.xlim(220,240)
    threshold = 0.3
    cylinders = []
    n_indices = 0
    avg_depth= 0 
    avg_indice = 0 
    start = True
    for i in range(len(derivative)):
        if derivative[i] < -threshold :
            n_indices = 0
            avg_depth= 0 
            avg_indice = 0
            start = True
        if derivative[i] > threshold and n_indices > 0:
            avg_indice  = avg_indice / n_indices
            avg_depth = avg_depth / n_indices
            cylinders.append([avg_depth, avg_indice])
            start = False
            print(avg_depth)
        if start == True:
            avg_indice += i
            n_indices += 1
            avg_depth += lidar_data[i+1][1]
    
    for i in range(len(cylinders)):
        plt.plot(cylinders[i][1], cylinders[i][0], marker = 'x', markersize = 20)
    print(cylinders)
    return cylinders

def rayInd2angle(ind):
    total_ind = 361
    angle = ind * math.pi / total_ind
    return angle
    
    
def plot_cylinders(cylinders, num_rays):
    
    cylinder_offset = 0.6
    X = []
    Y = []
    for i in range(len(cylinders)):
        theta = rayInd2angle(cylinders[i][1])
        r = cylinders[i][0] + cylinder_offset
        
        x= r * math.sin(theta)
        y = r * math.cos(theta)
        X.append(x)
        Y.append(y)
    plotCoord()
    fig= plt.scatter(X, Y, marker = 'x')
    
    return 
    
    
if __name__ == '__main__':
    lidar_data = process_data()
    #plotCoord()
    #plotVsRay()
    cylinders = compute_derivative(lidar_data)
    plot_cylinders(cylinders, 711)