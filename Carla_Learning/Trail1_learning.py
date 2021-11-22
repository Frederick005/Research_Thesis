#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Allows controlling a vehicle with a keyboard. For a simpler and more
# documented example, please take a look at tutorial.py.

"""
Welcome to CARLA manual control.

Use ARROWS or WASD keys for control.

    W            : throttle
    S            : brake
    A/D          : steer left/right
    Q            : toggle reverse
    Space        : hand-brake
    P            : toggle autopilot
    M            : toggle manual transmission
    ,/.          : gear up/down
    CTRL + W     : toggle constant velocity mode at 60 km/h

    L            : toggle next light type
    SHIFT + L    : toggle high beam
    Z/X          : toggle right/left blinker
    I            : toggle interior light

    TAB          : change sensor position
    ` or N       : next sensor
    [1-9]        : change to sensor [1-9]
    G            : toggle radar visualization
    C            : change weather (Shift+C reverse)
    Backspace    : change vehicle

    R            : toggle recording images to disk

    CTRL + R     : toggle recording of simulation (replacing any previous)
    CTRL + P     : start replaying last recorded simulation
    CTRL + +     : increments the start time of the replay by 1 second (+SHIFT = 10 seconds)
    CTRL + -     : decrements the start time of the replay by 1 second (+SHIFT = 10 seconds)

    F1           : toggle HUD
    H/?          : toggle help
    ESC          : quit
"""

from __future__ import print_function


# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys
import time
#import numpy as np

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

from carla import ColorConverter as cc

def creating_client_map():
    client = carla.Client("localhost", 2000)
    client.set_timeout(20.0)
    world=client.get_world()
    print(client.get_available_maps())
    print(world)
    return(world,client)

#Trying to spawn actors.
def actorcreation(world,client,actorlist):
    blueprintlibrary = world.get_blueprint_library()
    vehicle_bp = blueprintlibrary.filter('cybertruck')[0]
    transform= carla.Transform(carla.Location(x=130,y=195,z=40),carla.Rotation(yaw=180))
    vehicle = world.try_spawn_actor(vehicle_bp,transform)
    #print("the seeing ehat the vehicle blueprint returns", blueprintlibrary)
    actorlist.append(vehicle)

    #adding a lidar actor
    lidar_bp = blueprintlibrary.find('sensor.lidar.ray_cast')
    lidar_bp.set_attribute('channels','32')
    lidar_bp.set_attribute('range', '20')
    lidar_bp.set_attribute('points_per_second', '56000')
    lidar_bp.set_attribute('rotation_frequency', str(40))
    lidar_location = carla.Location(0,0,2)
    lidar_rotation = carla.Rotation(0,0,0)
    lidar_transform = carla.Transform(lidar_location,lidar_rotation)
    lidar = world.spawn_actor(lidar_bp,lidar_transform,attach_to=vehicle)
    lidar.listen(lambda point_cloud: point_cloud.save_to_disk(r'D:\Fh-dortmund\Research_Thesis\%.6d.ply' % point_cloud.frame))
    actorlist.append(lidar)
    return actorlist

def Destroy_actor(client,actorlist):
    print("Destroying all actors muaha muaha muahahahahahahhaha")
    for actor in actorlist:
        print(actor)
    client.apply_batch([carla.command.DestroyActor(actor)for actor in actorlist])


def main():
    actorlist=[]
    try:
        world,client= creating_client_map()
        actorlist=actorcreation(world,client,actorlist)
        print(actorlist)
        time.sleep(15)
    finally:
        Destroy_actor(client,actorlist)
        print ("destroyed")

if __name__ == '__main__':
    main()
