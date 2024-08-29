import glob
import os
import sys
import time
import random
import argparse
from collections import OrderedDict
import logging
import math
from sklearn.cluster import DBSCAN
from pandas import DataFrame
try:
    sys.path.append(glob.glob('C:\RThesis\Delete\carla\PythonAPI\carla\dist\carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import math
from carla import VehicleLightState as vls
import matplotlib.pyplot as plt
from queue import Queue
import numpy as np
from copy import copy
from matplotlib.patches import Rectangle


NUM_VEHICLES = 50
NUM_PEDESTRIANS = 40
VELOCITY_THRESHOLD= 0.20 #m/s
FRAMES_TRIMMED = 100
velocity_list = []
# Variable to count the number of radar frames received
radar_frames_count = 0

# Variable to count the number of the frame. delete after use. its a means to an end.
l = 0

# Cartesian coordinate system 
point_x=[]
point_y=[]
point_vel = []
point_obj_id = []
point_range = []
uf_point_x=[]
uf_point_y=[]
uf_point_vel = []
uf_point_obj_id = []
uf_point_range = []

# Spherical coordinate system only for plotting
sp_point_x=[]
sp_point_y=[]
sp_point_range = []
sp_uf_point_x=[]
sp_uf_point_y=[]
sp_uf_point_range = []
 
radar_queue=Queue()

#delete variables

#Function to perform DBSCAN clustering
def dbscan_clustering(max_point_x, max_point_y):
    r"""
    Function for performing DBSCAN clustering on  list of moving detection points available after 
    seperating moving and stationary points

    :param max_point_x: list
        list of the x axis points of the moving detections
    :param max_point_y: list
        list of the y axis points of the moving detections

    :return: 
        cluster: contains the detection points clustered by the DBSCAN algorithm
        array_in: contains the x and y points combined and stored in an array
    """
    array_in=np.empty([len(max_point_x),2])
    #Combining the x and y points received in into an numpy array of format n:2
    for i, detection in enumerate(max_point_x):
        array_in[i][0] = detection
        array_in[i][1] = max_point_y[i]
    #print(max_point_x)
    #print('Array containing x and y points',array_in)
    dummy = np.empty(shape = array_in.shape) 
    if array_in.size != 0:        
        clustering = DBSCAN(eps=3,min_samples=2).fit(array_in) 
        cluster = clustering.labels_
        return cluster,array_in
    else:
        return dummy 
    
    #Function to perform DBSCAN clustering
def dbscan_clustering3D(max_point_x, max_point_y,max_point_range):
    r"""
    Function for performing DBSCAN clustering on list of moving detection points available after 
    seperating moving and stationary points

    :param max_point_x: list
        list of the x axis points of the moving detections
    :param max_point_y: list
        list of the y axis points of the moving detections

    :return: 
        cluster: contains the detection points clustered by the DBSCAN algorithm
        array_in: contains the x and y points combined and stored in an array
    """
    # Creating a numpy array to store the position of points. 
    array_in=np.empty([len(max_point_x),3])

    #Combining the x, y and z points received in into an numpy array of format n:3
    for i, detection in enumerate(max_point_x):
        array_in[i][0] = detection
        array_in[i][1] = max_point_y[i]
        array_in[i][2] = max_point_range[i]

    # Array to be returned in case there is some data missing
    dummy = np.empty(shape = array_in.shape) 

    if array_in.size != 0:    
        # Creating the DBSCAn object and fitting model to database.     
        clustering = DBSCAN(eps=2,min_samples=2).fit(array_in) 
        # Cluster contains the labels assigned to each point. 
        cluster = clustering.labels_
        return cluster,array_in
    else:
        return dummy 

def process_data(flag_plt):
    r"""
    Funtion to extract the frames with the most moving detection points and perform DBSCAN clustereing on 
    points in that frame
    """
    max_count = 0
    max = 3
    count = 0
        
    # Adding a section of code which checks the diversity of object ids in the list. This is just 
    # for this project presentation. To get the most optimum frame. 
    for frame in point_obj_id:
        my_dict = {i:frame.count(i) for i in frame}
        print("the vehiles found", my_dict)
        if len(my_dict) == 3:
            print ("Found ONE OBJECT")
            fl = 0
            for key in my_dict:
                print (key)
                #fl = 0
                if my_dict[key] >= 3 and key != 0:
                    print("Found more than 3 points for an object")
                    fl = 1
                else:
                    fl = 0
                    break
            if fl == 1:
                print("this is the latest")
                max_count = count
        count += 1

    #plot_radar_processing(flag_plt, max_count)
    #plot_radar_processing(max_uf_point_x, max_uf_point_y, max_uf_point_range)
    cluster_mov, array_mov = dbscan_clustering3D(point_x[max_count], point_y[max_count], point_range[max_count])
    cluster_stat, array_stat = dbscan_clustering3D(uf_point_x[max_count], uf_point_y[max_count], uf_point_range[max_count])
    #print ('Cluster', cluster)
    # Printing both the stationary object clusters and the moving point clusters in one plot as individual subplots
    fig = plt.figure(figsize = (15, 15))
    ax = fig.add_subplot(1, 2, 1, projection='3d')
    ax = show_clusters3dcolor(array_mov,cluster_mov,ax ,frameNo=1,flag=2)
    ax = fig.add_subplot(1, 2, 2, projection='3d')
    ax = show_clusters3dcolor(array_stat,cluster_stat,ax ,frameNo=1,flag=3)
    plt.show() 


def plot_radar_processing(flag_plt, max_count):
    r"""
    Function for plotting graphs for the radar processing. This includes plotting graphs for conversion from
    spherical to cartesian coordinate system. Also, plots the stationary and moving objects. It does all this
    based on the 'flag_plt' variable. 

    :param flag_plt: int
        contains flag value
    :param max_count: int
        contains the frame with the maximum number of points. 

    :return: 
        nothing
    """
    if flag_plt == 1: 
        # creating an empty figure for plotting
        fig = plt.figure(figsize = (15, 15))

        # defining a sub-plot with 1x2 axis and defining 
        # it as first plot with projection as 3D
        ax = fig.add_subplot(1, 2, 1, projection='3d')
        ax.set_xlabel(r'Azimuth ($\phi$)', fontweight ='bold') 
        ax.set_ylabel('Range (r)', fontweight ='bold') 
        ax.set_zlabel(r'Elevation ($\theta$)', fontweight ='bold')
        ax.set_xlim([-15,15])
        ax.set_zlim(-15,15)
        ax.set_ylim([-0,100])
        ax.set_title("Detections in Spherical Coordinate System")
        ax.scatter3D(sp_point_x[max_count], sp_point_range[max_count], sp_point_y[max_count], color = "green")
        ax.scatter3D(sp_uf_point_x[max_count], sp_uf_point_range[max_count], sp_uf_point_y[max_count], color = "green", label='Radar detections')
        ax.legend(loc = 2)

        # Creating my second subplot with 1x2 axis and defining 
        # it as the second plot with projection as 3D
        ax = fig.add_subplot(1, 2, 2, projection='3d')
        ax.set_xlabel('Latitude (x, in Meter)', fontweight ='bold') 
        ax.set_ylabel('Depth (z, in Meter)', fontweight ='bold') 
        ax.set_zlabel('Longitude (y, in Meter)', fontweight ='bold')
        ax.set_xlim([-14,23])
        ax.set_zlim(-2,10)
        ax.set_ylim([-0,100])
        ax.set_title("Detections in Cartesian Coordinate System")
        ax.scatter3D(point_x[max_count], point_range[max_count], point_y[max_count], color = "green")
        ax.scatter3D(uf_point_x[max_count], uf_point_range[max_count], uf_point_y[max_count], color = "green", label='Radar detections')
        ax.legend(loc = 2)

        plt.show()

    elif flag_plt == 2:
        # creating an empty figure for plotting
        fig = plt.figure(figsize = (15, 15))

        # defining a sub-plot with 1x2 axis and defining 
        # it as first plot with projection as 3D
        ax = fig.add_subplot(1, 2, 1, projection='3d')
        ax.set_xlabel('Latitude (x, in Meter)', fontweight ='bold') 
        ax.set_ylabel('Depth (z, in Meter)', fontweight ='bold') 
        ax.set_zlabel('Longitude (y, in Meter)', fontweight ='bold')
        ax.set_xlim([-14,23])
        ax.set_zlim(-2,10)
        ax.set_ylim([-0,100])
        ax.set_title("Detections prior to Filtering")
        ax.scatter3D(point_x[max_count], point_range[max_count], point_y[max_count], color = "green")
        ax.scatter3D(uf_point_x[max_count], uf_point_range[max_count], uf_point_y[max_count], color = "green", label='Radar detections')
        ax.legend(loc = 2)

        # Creating my second subplot with 1x2 axis and defining 
        # it as the second plot with projection as 3D
        ax = fig.add_subplot(1, 2, 2, projection='3d')
        ax.set_xlabel('Latitude (x, in Meter)', fontweight ='bold') 
        ax.set_ylabel('Depth (z, in Meter)', fontweight ='bold') 
        ax.set_zlabel('Longitude (y, in Meter)', fontweight ='bold')
        ax.set_xlim([-14,23])
        ax.set_zlim(-2,10)
        ax.set_ylim([-0,100])
        ax.set_title("Detections after Filtering")
        ax.scatter3D(point_x[max_count], point_range[max_count], point_y[max_count], color = "red", marker = '*', label='Moving points')
        ax.scatter3D(uf_point_x[max_count], uf_point_range[max_count], uf_point_y[max_count], color = "green", label='Stationary Points')
        ax.legend(loc = 2)

        plt.show()


def show_clusters3d(array_cluster,cluster,frameNo,flag=0):  
    df = DataFrame(dict(x=array_cluster[:,0],y=array_cluster[:,1],label=cluster))   
    unique_labels = set(cluster)    
    colors = [plt.cm.Spectral(each)
              for each in np.linspace(0, 1, len(unique_labels))]  
    print('colors of points', colors)
    fig = plt.figure(figsize = (8, 8))
    ax = fig.add_subplot(projection='3d')
    ax.set_xlabel('Latitude (x, in Meter)', fontweight ='bold') 
    ax.set_ylabel('Depth (z, in Meter)', fontweight ='bold') 
    ax.set_zlabel('Longitude (y, in Meter)', fontweight ='bold')
    ax.set_xlim([-14,23])
    ax.set_zlim(-2,10)
    ax.set_ylim([-0,100])
    if flag == 0:
        plt.title(' Null')
    if flag == 1:
        plt.title(' All Detection Points')
    if flag == 2:
        plt.title('Moving points')
    if flag == 3:
        plt.title('Stationary points')        
    #plt.xlim([-17,17])
    
    #ax.scatter3D(array_cluster[:, 0], array_cluster[:, 2], array_cluster[:, 1], c=cluster, marker='o')

    plt.grid(True)  
    plt.legend(loc = 'upper right')
  
    # if flag == 2:        
    #     plt.savefig('Moving_{}.png'.format(frameNo), dpi=300, bbox_inches='tight') 
    # if flag == 3:        
    #     plt.savefig('Ambig_{}.png'.format(frameNo), dpi=300, bbox_inches='tight')   
    plt.show() 

def show_clusters3dcolor(array_cluster,cluster, ax,frameNo,flag=0):  
    unique_labels = set(cluster)    
    print('unique clusters', unique_labels)
    colors = [plt.cm.Spectral(each)
              for each in np.linspace(0, 1, len(unique_labels))]  
    print('colors of points', colors)
    ax.set_xlabel('Latitude (x, in Meter)', fontweight ='bold') 
    ax.set_ylabel('Depth (z, in Meter)', fontweight ='bold') 
    ax.set_zlabel('Longitude (y, in Meter)', fontweight ='bold')
    ax.set_xlim([-14,23])
    ax.set_zlim(-2,10)
    ax.set_ylim([-0,100])
    if flag == 0:
        plt.title(' Null')
    if flag == 1:
        plt.title(' All Detection Points')
    if flag == 2:
        plt.title('Moving points')
    if flag == 3:
        plt.title('Stationary points')        

    for k, col in zip(unique_labels, colors):
        if k == -1:
            # Black used for noise.
            col = [0, 0, 0, 1]

        class_index = np.where(cluster == k)[0]
        for ci in class_index:
            ax.plot(
                array_cluster[ci, 0],
                array_cluster[ci, 2],
                array_cluster[ci, 1],
                marker = "x" if k == -1 else "d" if flag == 3 else "o",
                markerfacecolor=tuple(col),
                markeredgecolor="k",
                label = "Noise" if k == -1 else "Stationary Cluster {}".format(k+1) if flag == 3 else "Moving Cluster {}".format(k+1)
            )
    plt.grid(True)  
    #plt.legend(loc = 'upper right', numpoints = 1)
    handles, labels = plt.gca().get_legend_handles_labels()
    by_label = OrderedDict(zip(labels, handles))
    plt.legend(by_label.values(), by_label.keys())
  
    # if flag == 2:        
    #     plt.savefig('Moving_{}.png'.format(frameNo), dpi=300, bbox_inches='tight') 
    # if flag == 3:        
    #     plt.savefig('Ambig_{}.png'.format(frameNo), dpi=300, bbox_inches='tight')   
    return ax
    


def show_clusters(array_cluster,cluster,frameNo,flag=0):  
    df = DataFrame(dict(x=array_cluster[:,0],y=array_cluster[:,1],label=cluster))   
    unique_labels = set(cluster)    
    colors = [plt.cm.Spectral(each)
              for each in np.linspace(0, 1, len(unique_labels))]  
    print('colors of points', colors)
      
    fig,ax = plt.subplots(figsize=(8,8))
    plt.ylabel('Longitude (y, in Meter)')    
    plt.xlabel('Latitude (x, in Meter)') 
    if flag == 0:
        plt.title(' Null')
    if flag == 1:
        plt.title(' All Detection Points')
    if flag == 2:
        plt.title(' Frame {} - Moving points'.format(frameNo))
    if flag == 3:
        plt.title(' Frame {} - Stationary points'.format(frameNo))        
    #plt.xlim([-17,17])
    plt.xlim([-15,15])
    plt.ylim([-15,15])
            
    grouped = df.groupby('label')    
    for key,group in grouped:        
        group.plot(ax=ax, kind = 'scatter', x='x',y='y', label=key, color = colors[key])
    
    rect = get_box(cluster,array_cluster,flag)    
    for c in rect:
        new_c=copy(c)
        ax.add_patch(new_c)    

    plt.grid(True)  
    plt.legend(loc = 'upper right')
  
    # if flag == 2:        
    #     plt.savefig('Moving_{}.png'.format(frameNo), dpi=300, bbox_inches='tight') 
    # if flag == 3:        
    #     plt.savefig('Ambig_{}.png'.format(frameNo), dpi=300, bbox_inches='tight')   
    plt.show() 
    df.groupby('label')   


""" 
    get_box(cluster,array_cluster,flag)
    This function makes boxes around the clustered points
"""        
def get_box(cluster,array_cluster,flag):
    if -1 in cluster:
        cluster_num = len(set(cluster))-1
    else:
        cluster_num = len(set(cluster))    
    rect = []
    width = 0
    height = 0
    
    for i in range(cluster_num): 
        box_x = []
        box_y = []
        for k, cluster_pt in enumerate(cluster):            
            if cluster_pt == i:                
                box_y.append(array_cluster[k][1])
                box_x.append(array_cluster[k][0])              

        width = max(box_x) - min(box_x)
        height = max(box_y) - min(box_y)        
        #if flag == 1:
            #rect.append(Rectangle((min(box_x),min(box_y)), width, height, fill=False, color='blue'))
        if flag == 2:
            rect.append(Rectangle((min(box_x),min(box_y)), width, height, fill=False, color='green'))
        if flag == 3:            
            rect.append(Rectangle((min(box_x),min(box_y)), width, height, fill=False, color='red'))     
        
    return rect  


# Creating a function to see if we can resolve the vector so that we can get the forward moving velocity of the target vehicle to filter stationary points
def rad_callback_vectorres(radar_data, vehicle,vehicle_target_1,vehicle_target_2):
    i = 0
    # Initially just checking to see if the velocity can be taken out. what we need for it:
    ###### lets check for 15 data so that we are not overwhelmed
    ###### split it into its resultant vectors to get the forwards moving obgect and also see if  stationary and moving objects have different speeds
    for detect in radar_data:
        radial_vel = detect.velocity
        
        i += 1
        azi = detect.azimuth
        target_speed= radial_vel * math.cos(azi)
        print("velocity", vehicle.get_velocity())
        print("velocity_target 1", vehicle_target_1.get_velocity())
        print("velocity_target 2", vehicle_target_2.get_velocity())
        if detect.obj_id == 0:
            print ('For stationary objects ')
            print ('Azimuth', azi)
            print ('radial velocity', radial_vel)
            print ('Target Speed', target_speed ,'\n')
        else:
            print ('For moving objects ')
            print ('Azimuth', azi)
            print ('radial velocity', radial_vel)
            print ('Target Speed', target_speed ,'\n')
        if i>15:
            print ('NEW ROUND'* 5)
            break
    print("number of detections", i)
    global radar_frames_count
    radar_frames_count += 1


# Function to store the data the we listen from the radar to a queue. 
################################################################################
# It is better to store the radar data first before processing it because before 
# we can finish processing it new data arrives via the radar.listen()
def radar_callback_queue(radar_data, radar_queue, vehicle):
    r"""
    Function to store the data the we listen from the radar to a queue. 
    Also store the velocity of the EGO vehicle at each frame into a list

    :param radar_data: 
        It is the rawdata that is received from the radar.listen lambda function
    
    :param radar_queue: Queue
        Queue to store the data received each time in the radar.listen function

    :param vehicle: 
        the variable to store the spawned ego vehicle so that we can access the speed at each frame

    :return: NONE (Lambda function)
    """
    radar_queue.put(radar_data)
    velocity_list.append(abs(vehicle.get_velocity().x))


# Function to process the queue that was received via the radar listen function.
################################################################################
def radar_data_processing():
    radar_data= []
    while not radar_queue.empty():
        radar_data.append(radar_queue.get())

    # Checking if we have the velocity of the vehicle available for each frame
    # This velocity list will be helpful in filtering the moving points from the stationary points
    if len(velocity_list)== len(radar_data):
        print('\nWe have the respective velocity for each frame\n')
    else:
        print('Mismatch in the number of velocities and frames')

    # Trimming the radar data list
    temp = radar_data[FRAMES_TRIMMED:]
    del temp[-FRAMES_TRIMMED:]
    radar_data_1=temp

    # Trimming the velocity list
    temp = velocity_list[FRAMES_TRIMMED:]
    del temp[-FRAMES_TRIMMED:]
    velocity_list_1=temp
    print('Number of frames left after trimming',len(radar_data_1))
    print('####'*20)
    # Filtering the points for each frame
    vel_count = 0
    for frame in radar_data_1:
        filtering_moving_points(frame, velocity_list_1[vel_count])
        #Get_the_fov(frame, velocity_list_1[vel_count])
        vel_count += 1
    #get_fov()
    return radar_data

# Function to filter out the moving points from stationary points 
def filtering_moving_points(frame, ego_velocity):
    interm_point_x = []
    interm_point_y = []
    interm_obj_id = []
    interm_point_vel = []
    interm_range = []
    interm_uf_point_x=[]
    interm_uf_point_y=[]
    interm_uf_point_obj_id = []
    interm_uf_point_range = []

    interm_sp_point_x = []
    interm_sp_point_y = []
    interm_sp_range = []
    interm_sp_uf_point_x=[]
    interm_sp_uf_point_y=[]
    interm_sp_uf_point_range = []
    current_rot = frame.transform.rotation
    for point in frame:
        azi = point.azimuth

        # Calculating target velocity
        radial_vel = point.velocity
        target_speed= radial_vel * math.cos(azi)
        if point.obj_id != 0:
            print ("target vel:", target_speed)
            print ("ego vel:", ego_velocity)
        #print (point)
            
        # Converting points stored in spherical coordinate system to cartesian. 
        azi = math.degrees(point.azimuth)
        alt = math.degrees(point.altitude)
        # The 0.25 adjusts a bit the distance so the dots can
        # be properly seen
        fw_vec = carla.Vector3D(x=point.depth - 0.25)
        carla.Transform(
            carla.Location(),
            carla.Rotation(
                pitch=current_rot.pitch + alt,
                yaw=current_rot.yaw + azi,
                roll=current_rot.roll)).transform(fw_vec)
        
        if abs(target_speed)<= ego_velocity+ VELOCITY_THRESHOLD and abs(target_speed)>= ego_velocity-VELOCITY_THRESHOLD:
            interm_sp_uf_point_x.append(math.degrees(point.azimuth))
            interm_sp_uf_point_y.append(math.degrees(point.altitude))
            interm_sp_uf_point_range.append(point.depth)

            interm_uf_point_x.append(-fw_vec.y)
            interm_uf_point_y.append(fw_vec.z)
            interm_uf_point_obj_id.append(point.obj_id)
            interm_uf_point_range.append(-fw_vec.x)
        else:
            interm_sp_point_x.append(math.degrees(point.azimuth))
            interm_sp_point_y.append(math.degrees(point.altitude))
            interm_sp_range.append(point.depth)

            interm_point_x.append(-fw_vec.y)
            interm_point_y.append(fw_vec.z)
            interm_obj_id.append(point.obj_id)
            interm_range.append(-fw_vec.x)
    point_x.append(interm_point_x)
    point_y.append(interm_point_y)
    point_obj_id.append(interm_obj_id)
    point_range.append(interm_range)
    uf_point_x.append(interm_uf_point_x)
    uf_point_y.append(interm_uf_point_y)
    uf_point_obj_id.append(interm_uf_point_obj_id)
    uf_point_range.append(interm_uf_point_range)

    sp_point_x.append(interm_sp_point_x)
    sp_point_y.append(interm_sp_point_y)
    sp_point_range.append(interm_sp_range)
    sp_uf_point_x.append(interm_sp_uf_point_x)
    sp_uf_point_y.append(interm_sp_uf_point_y)
    sp_uf_point_range.append(interm_sp_uf_point_range)


def rad_callback(radar_data, world):
    c=0
    interm_point_x = []
    interm_point_y = []
    interm_obj_id = []
    for detect in radar_data:
        if detect.obj_id != 0:
            azi = math.degrees(detect.azimuth)
            alt = math.degrees(detect.altitude)
            interm_point_x.append(azi)
            interm_point_y.append(alt)
            interm_obj_id.append(detect.obj_id)
            c +=1
    print ("Number of points", c)
    point_x.append(interm_point_x)
    point_y.append(interm_point_y)
    point_obj_id.append(interm_obj_id)


def rad_callback_visualization(radar_data,world):
    velocity_range = 7.5 # m/s
    current_rot = radar_data.transform.rotation
    c=0
    interm_point_x = []
    interm_point_y = []
    interm_obj_id = []
    for detect in radar_data:
        radial_vel = detect.velocity
        azi = detect.azimuth
        alt = detect.altitude
        target_speed= radial_vel * math.cos(azi)

        # Lets check the velocity of the vehicle every 20 frames so that it is not a burden

        if detect.obj_id != 0:
            interm_point_x.append(azi)
            interm_point_y.append(alt)
            #interm_obj_id.append(detect.obj_id)
            c +=1

        # The 0.25 adjusts a bit the distance so the dots can
        # be properly seen
        fw_vec = carla.Vector3D(x=detect.depth - 0.25)
        carla.Transform(
            carla.Location(),
            carla.Rotation(
                pitch=current_rot.pitch + alt,
                yaw=current_rot.yaw + azi,
                roll=current_rot.roll)).transform(fw_vec)

        def clamp(min_v, max_v, value):
            return max(min_v, min(value, max_v))

        norm_velocity = detect.velocity / velocity_range # range [-1, 1]
        r = int(clamp(0.0, 1.0, 1.0 - norm_velocity) * 255.0)
        g = int(clamp(0.0, 1.0, 1.0 - abs(norm_velocity)) * 255.0)
        b = int(abs(clamp(- 1.0, 0.0, - 1.0 - norm_velocity)) * 255.0)
        world.debug.draw_point(
            radar_data.transform.location + fw_vec,
            size=0.075,
            life_time=0.06,
            persistent_lines=False,
            color=carla.Color(r, g, b))
    print ("Number of points", c)
    point_x.append(interm_point_x)
    point_y.append(interm_point_y)
    point_obj_id.append(interm_obj_id)



def scenario_1_car_car(world, actorlist):
    # Creating 2 other cars in front of the ego vehicle so that the radar can be tested
    ###################################################################################
    # First car
    blueprintlibrary = world.get_blueprint_library()
    vehicle_target_1_bp = blueprintlibrary.filter("etron")[0]
    transform_target_1 = carla.Transform(carla.Location(x=100, y=196, z=4), carla.Rotation(yaw=180))
    vehicle_target_1 = world.try_spawn_actor(vehicle_target_1_bp, transform_target_1)
    #vehicle_target_1.set_autopilot(True)
    actorlist.append(vehicle_target_1)

    # Second car
    vehicle_target_2_bp = blueprintlibrary.filter("etron")[0]
    transform_target_2 = carla.Transform(carla.Location(x=100, y=192, z=4), carla.Rotation(yaw=180))
    vehicle_target_2 = world.try_spawn_actor(vehicle_target_2_bp, transform_target_2)
    #vehicle_target_2.set_autopilot(True)
    actorlist.append(vehicle_target_2)

    # Creating the Ego vehicle
    ##########################
    vehicle_bp = blueprintlibrary.filter('cybertruck')[0]
    transform = carla.Transform(carla.Location(x=130, y=194, z=4), carla.Rotation(yaw=180))
    vehicle = world.try_spawn_actor(vehicle_bp, transform)
    #vehicle.set_autopilot(True)
    actorlist.append(vehicle)

    vehicle_target_1.apply_control(carla.VehicleControl(throttle = 0.3, brake =0))
    vehicle_target_2.apply_control(carla.VehicleControl(throttle = 0.3, brake =0))
    vehicle.apply_control(carla.VehicleControl(throttle = 0.3, brake =0))

    # Mounting the radar on the Ego vehicle
    #######################################w
    rad_bp = world.get_blueprint_library().find('sensor.other.radar')
    #rad_bp.set_attribute('horizontal_fov', str(35))
    #rad_bp.set_attribute('vertical_fov', str(20))
    rad_bp.set_attribute('range', str(100))
    rad_bp.set_attribute('points_per_second', str(15000))
    rad_bp.set_attribute('sensor_tick', str(10.0))
    rad_location = carla.Location(x=2.0, z=1.0)
    rad_rotation = carla.Rotation(pitch=0)
    rad_transform = carla.Transform(rad_location,rad_rotation)
    rad_ego = world.spawn_actor(rad_bp,rad_transform,attach_to=vehicle, attachment_type=carla.AttachmentType.Rigid)
    actorlist.append(rad_ego)


    # Listening to radar which is attached on the EGO vehicle
    #########################################################

    #For queue function
    rad_ego.listen(lambda rad_data: radar_callback_queue(rad_data, radar_queue,vehicle))

    time.sleep(9)


def scenario_2_car_cycle(world, actorlist):
    # Creating 1 car and 1 cycle in front of the ego vehicle so that the radar can be tested
    ###################################################################################
    # First car
    blueprintlibrary = world.get_blueprint_library()
    vehicle_target_1_bp = blueprintlibrary.filter("etron")[0]
    transform_target_1 = carla.Transform(carla.Location(x=100, y=196, z=3), carla.Rotation(yaw=180))
    vehicle_target_1 = world.try_spawn_actor(vehicle_target_1_bp, transform_target_1)
    #vehicle_target_1.set_autopilot(True)
    actorlist.append(vehicle_target_1)

    # Bike
    vehicle_target_2_bp = blueprintlibrary.filter("crossbike")[0]
    transform_target_2 = carla.Transform(carla.Location(x=100, y=192, z=2), carla.Rotation(yaw=180))
    vehicle_target_2 = world.try_spawn_actor(vehicle_target_2_bp, transform_target_2)
    #vehicle_target_2.set_autopilot(True)
    actorlist.append(vehicle_target_2)

    # Creating the Ego vehicle
    ##########################
    vehicle_bp = blueprintlibrary.filter('cybertruck')[0]
    transform = carla.Transform(carla.Location(x=130, y=194, z=3), carla.Rotation(yaw=180))
    vehicle = world.try_spawn_actor(vehicle_bp, transform)
    #vehicle.set_autopilot(True)
    actorlist.append(vehicle)

    vehicle_target_1.apply_control(carla.VehicleControl(throttle = 0.3, brake =0))
    vehicle_target_2.apply_control(carla.VehicleControl(throttle = 0.3, brake =0))
    vehicle.apply_control(carla.VehicleControl(throttle = 0.3, brake =0))

    # Mounting the radar on the Ego vehicle
    #######################################w
    rad_bp = world.get_blueprint_library().find('sensor.other.radar')
    #rad_bp.set_attribute('horizontal_fov', str(35))
    #rad_bp.set_attribute('vertical_fov', str(20))
    rad_bp.set_attribute('range', str(100))
    rad_bp.set_attribute('points_per_second', str(15000))
    rad_bp.set_attribute('sensor_tick', str(10.0))
    rad_location = carla.Location(x=2.0, z=1.0)
    rad_rotation = carla.Rotation(pitch=0)
    rad_transform = carla.Transform(rad_location,rad_rotation)
    rad_ego = world.spawn_actor(rad_bp,rad_transform,attach_to=vehicle, attachment_type=carla.AttachmentType.Rigid)
    actorlist.append(rad_ego)


    # Listening to radar which is attached on the EGO vehicle
    #########################################################

    #For queue function
    rad_ego.listen(lambda rad_data: radar_callback_queue(rad_data, radar_queue,vehicle))

    time.sleep(9)


def scenario_3_car_pedestrian(world, actorlist):
    # Creating 2 other cars in front of the ego vehicle so that the radar can be tested
    ###################################################################################
    # First car
    SpawnActor = carla.command.SpawnActor
    blueprintlibrary = world.get_blueprint_library()
    vehicle_target_1_bp = blueprintlibrary.filter("etron")[0]
    transform_target_1 = carla.Transform(carla.Location(x=100, y=196, z=3), carla.Rotation(yaw=180))
    vehicle_target_1 = world.try_spawn_actor(vehicle_target_1_bp, transform_target_1)
    #vehicle_target_1.set_autopilot(True)
    actorlist.append(vehicle_target_1)

    # Pedestrian
    walker_bp = blueprintlibrary.find('walker.pedestrian.0002')
    walker_bp.set_attribute('speed', '10')
    transform_target_2 = carla.Transform(carla.Location(x=105, y=192, z=2), carla.Rotation(yaw=180))
    walker1= world.try_spawn_actor(walker_bp, transform_target_2)
    actorlist.append(walker1)

    # Creating the Ego vehicle
    ##########################
    vehicle_bp = blueprintlibrary.filter('cybertruck')[0]
    transform = carla.Transform(carla.Location(x=130, y=194, z=3), carla.Rotation(yaw=180))
    vehicle = world.try_spawn_actor(vehicle_bp, transform)
    #vehicle.set_autopilot(True)
    actorlist.append(vehicle)

    vehicle_target_1.apply_control(carla.VehicleControl(throttle = 0.3, brake =0))
    walker1.apply_control(carla.WalkerControl(direction=carla.Vector3D(-1.0,0.0,0.0), speed = 2))
    vehicle.apply_control(carla.VehicleControl(throttle = 0.3, brake =0))

    # Mounting the radar on the Ego vehicle
    #######################################w
    rad_bp = world.get_blueprint_library().find('sensor.other.radar')
    #rad_bp.set_attribute('horizontal_fov', str(35))
    #rad_bp.set_attribute('vertical_fov', str(20))
    rad_bp.set_attribute('range', str(100))
    rad_bp.set_attribute('points_per_second', str(15000))
    rad_bp.set_attribute('sensor_tick', str(10.0))
    rad_location = carla.Location(x=2.0, z=1.0)
    rad_rotation = carla.Rotation(pitch=0)
    rad_transform = carla.Transform(rad_location,rad_rotation)
    rad_ego = world.spawn_actor(rad_bp,rad_transform,attach_to=vehicle, attachment_type=carla.AttachmentType.Rigid)
    actorlist.append(rad_ego)


    # Listening to radar which is attached on the EGO vehicle
    #########################################################

    #For queue function
    rad_ego.listen(lambda rad_data: radar_callback_queue(rad_data, radar_queue,vehicle))

    # For visualization in Carla
    # rad_ego.listen(lambda rad_data: rad_callback_visualization(rad_data, world))

    time.sleep(9)


def scenario_4_pedestrian_opposite(world,actorlist):
    # Creating 2 other cars in front of the ego vehicle and a pedestrian approaches 
    # opposite direction.
    ###################################################################################
    # First car
    blueprintlibrary = world.get_blueprint_library()
    vehicle_target_1_bp = blueprintlibrary.filter("etron")[0]
    transform_target_1 = carla.Transform(carla.Location(x=100, y=194, z=4), carla.Rotation(yaw=180))
    vehicle_target_1 = world.try_spawn_actor(vehicle_target_1_bp, transform_target_1)
    #vehicle_target_1.set_autopilot(True)
    actorlist.append(vehicle_target_1)

    # Second car
    vehicle_target_2_bp = blueprintlibrary.filter("Impala")[0]
    transform_target_2 = carla.Transform(carla.Location(x=100, y=190, z=4), carla.Rotation(yaw=180))
    vehicle_target_2 = world.try_spawn_actor(vehicle_target_2_bp, transform_target_2)
    #vehicle_target_2.set_autopilot(True)
    actorlist.append(vehicle_target_2)

    # Third car
    vehicle_target_3_bp = blueprintlibrary.filter("walker.pedestrian.0002")[0]
    vehicle_target_3_bp.set_attribute('speed', '10')
    transform_target_3 = carla.Transform(carla.Location(x=90, y=197, z=4), carla.Rotation(yaw=180))
    vehicle_target_3 = world.try_spawn_actor(vehicle_target_3_bp, transform_target_3)
    #vehicle_target_2.set_autopilot(True)
    actorlist.append(vehicle_target_3)

    # Creating the Ego vehicle
    ##########################
    vehicle_bp = blueprintlibrary.filter('cybertruck')[0]
    transform = carla.Transform(carla.Location(x=120, y=194, z=4), carla.Rotation(yaw=180))
    vehicle = world.try_spawn_actor(vehicle_bp, transform)
    #vehicle.set_autopilot(True)
    actorlist.append(vehicle)

    vehicle_target_1.apply_control(carla.VehicleControl(throttle = 0.3, brake =0))
    vehicle_target_2.apply_control(carla.VehicleControl(throttle = 0.3, brake =0))
    vehicle_target_3.apply_control(carla.WalkerControl(direction=carla.Vector3D(1.0,0.0,0.0), speed = 2))
    vehicle.apply_control(carla.VehicleControl(throttle = 0.3, brake =0))

    # Mounting the radar on the Ego vehicle
    #######################################w
    rad_bp = world.get_blueprint_library().find('sensor.other.radar')
    #rad_bp.set_attribute('horizontal_fov', str(35))
    #rad_bp.set_attribute('vertical_fov', str(20))
    rad_bp.set_attribute('range', str(100))
    rad_bp.set_attribute('points_per_second', str(15000))
    rad_bp.set_attribute('sensor_tick', str(10.0))
    rad_location = carla.Location(x=2.0, z=1.0)
    rad_rotation = carla.Rotation(pitch=0)
    rad_transform = carla.Transform(rad_location,rad_rotation)
    rad_ego = world.spawn_actor(rad_bp,rad_transform,attach_to=vehicle, attachment_type=carla.AttachmentType.Rigid)
    actorlist.append(rad_ego)


    # Listening to radar which is attached on the EGO vehicle
    #########################################################

    #For queue function
    rad_ego.listen(lambda rad_data: radar_callback_queue(rad_data, radar_queue,vehicle))

    # For getting list of objects
    #rad_ego.listen(lambda rad_data: radar_data_split(rad_data, world))

    time.sleep(9)


def scenario_5_perpendicular_traffic(world,actorlist):
    # Creating 2 other cars in front of the ego vehicle so that the radar can be tested
    ###################################################################################

    # First car
    blueprintlibrary = world.get_blueprint_library()
    vehicle_target_1_bp = blueprintlibrary.filter("etron")[0]
    transform_target_1 = carla.Transform(carla.Location(x=12, y=196, z=4), carla.Rotation(yaw=180))
    vehicle_target_1 = world.try_spawn_actor(vehicle_target_1_bp, transform_target_1)
    actorlist.append(vehicle_target_1)

    # Pedestrian
    pedestrian_bp = blueprintlibrary.filter("walker.pedestrian.0002")[0]
    pedestrian_bp.set_attribute('speed', '10')
    transform_target_p = carla.Transform(carla.Location(x=20, y=200, z=4), carla.Rotation(yaw=180))
    pedestrian_target = world.try_spawn_actor(pedestrian_bp, transform_target_p)
    #vehicle_target_2.set_autopilot(True)
    actorlist.append(pedestrian_target)

    # Creating the Ego vehicle
    ##########################
    vehicle_bp = blueprintlibrary.filter('cybertruck')[0]
    transform = carla.Transform(carla.Location(x=5, y=165, z=4), carla.Rotation(yaw=90))
    vehicle = world.try_spawn_actor(vehicle_bp, transform)
    #vehicle.set_autopilot(True)
    actorlist.append(vehicle)

    vehicle_target_1.apply_control(carla.VehicleControl(throttle = 0.3, brake =0))
    pedestrian_target.apply_control(carla.WalkerControl(direction=carla.Vector3D(-1.0,0.0,0.0), speed = 2))
    vehicle.apply_control(carla.VehicleControl(throttle = 0.3, brake =0))

    # Mounting the radar on the Ego vehicle
    #######################################w
    rad_bp = world.get_blueprint_library().find('sensor.other.radar')
    rad_bp.set_attribute('horizontal_fov', str(30))
    rad_bp.set_attribute('vertical_fov', str(30))
    rad_bp.set_attribute('range', str(100))
    rad_bp.set_attribute('points_per_second', str(15000))
    rad_bp.set_attribute('sensor_tick', str(10.0))
    rad_location = carla.Location(x=2.0, z=1.0)
    rad_rotation = carla.Rotation(pitch=0)
    rad_transform = carla.Transform(rad_location,rad_rotation)
    rad_ego = world.spawn_actor(rad_bp,rad_transform,attach_to=vehicle, attachment_type=carla.AttachmentType.Rigid)
    actorlist.append(rad_ego)

    # Listening to radar which is attached on the EGO vehicle
    #########################################################

    #For queue function
    rad_ego.listen(lambda rad_data: radar_callback_queue(rad_data, radar_queue,vehicle))

    # For visualization in Carla
    #rad_ego.listen(lambda rad_data: rad_callback_visualization(rad_data, world))
    

    # For getting list of objects
    #rad_ego.listen(lambda rad_data: radar_data_split(rad_data, world))

    time.sleep(7)


def main():
    #Setting actor list to keep track of the actors to be destroyed
    actorlist = []
    #count = 0
    client = carla.Client("localhost", 2000)
    client.set_timeout(20.0)
    world = client.get_world()

    # Establishing the try catch block
    try:
        print("Starting Scenario Creation")
        # Calling First scenario to check for 2 cars in front
        scenario_1_car_car(world, actorlist)
        #scenario_2_car_cycle(world, actorlist)
        #scenario_3_car_pedestrian(world,actorlist)
        #scenario_4_pedestrian_opposite(world,actorlist)
        #scenario_5_perpendicular_traffic(world,actorlist)

        
    finally:
        print("I am gonna do it.... yea do it... Kill all actors Muahahahahahaha")
        print("number of frames", radar_frames_count)
        client.apply_batch([carla.command.DestroyActor(x) for x in actorlist])
        radar_data_processing()
        process_data(1)
        #print(point_vel)
        #with open("Radar_PointCloud.txt", 'w') as f:
        #    for list in point_x:
        #        f.write ("POINT X AXIS \n")
        #        f.write (str(list))
        #        f.write ("\n POINT Y AXIS \n")
        #        f.write (str(point_y[count]))
        #        f.write ("\n Object ID \n")
        #        f.write (str(point_obj_id[count]))
        #        f.write ("\n")
        #        f.write ("###"*30)
        #        f.write ("\n")
        #        count +=1
        #    f.close()
        #plt.scatter(points2, points)
        #plt.show()

if __name__ == "__main__":
    main()