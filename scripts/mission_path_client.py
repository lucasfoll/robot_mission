#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Point
from robot_mission.srv import *

def GetWaypoints():
    wp_list = []
    wp_count = 0

    robot_name = raw_input("Qual robô receberá a missão?: ")

    while wp_count >= 0:
        print("Digite a {}ª coordenada.".format(wp_count + 1))
        x = float(raw_input("X: "))
        y = float(raw_input("Y: "))
        
        wp_list.append(Point())
        wp_list[wp_count].x = x
        wp_list[wp_count].y = y
        wp_list[wp_count].z = 0

        resp = raw_input("Deseja adicionar mais uma coordenada? Digite 'y' para SIM e 'n' para NÃO: ")
        if resp == 'y':
            wp_count += 1
            continue
        else:
            break
    
    return robot_name, wp_list


if __name__ == "__main__":

    rospy.loginfo('Waiting for mission_path')
    print('Waiting for mission_path')
    rospy.wait_for_service('mission_path')
    
    robot_name, waypoints = GetWaypoints()
    
    try:
        mission_path = rospy.ServiceProxy('mission_path', MissionPath)
        response = mission_path(robot_name, waypoints)
        print(response)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
