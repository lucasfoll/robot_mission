#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from std_msgs.msg import Empty
from robot_mission.srv import MissionPath, MissionPathResponse

def transform_msg(data):
    robot_name = data.robot_name
    waypoint_pub = rospy.Publisher('{}/wp_topic'.format(robot_name), PoseWithCovarianceStamped, queue_size=10)
    execute_path_pub = rospy.Publisher('{}/path_ready'.format(robot_name), Empty, queue_size=10)
    rospy.loginfo('Waypoints for {} recived'.format(robot_name))  
    i = 0
    j = 0
    waypoints_path = []

    for wp in data.waypoints:
        #Criando mensagem.
        waypoints_path.append(PoseWithCovarianceStamped())
        #Preenchendo o Header
        waypoints_path[i].header.stamp = rospy.Time.now()
        waypoints_path[i].header.frame_id = "map"
        #Preenchendo o Pose Point
        waypoints_path[i].pose.pose.position = wp
        #Preenchendo o Quartenion, pois nao passo essa informacao. Interessante add futuramente algo pratico que faca isso.
        waypoints_path[i].pose.pose.orientation.x = 0
        waypoints_path[i].pose.pose.orientation.y = 0
        waypoints_path[i].pose.pose.orientation.z = 0
        waypoints_path[i].pose.pose.orientation.w = 1

        #teste
        rospy.loginfo('contador: {}'.format(i))
        rospy.loginfo(waypoints_path[i])
        #fim teste

        i += 1

    amcl_pose = rospy.wait_for_message('{}/amcl_pose'.format(robot_name), PoseWithCovarianceStamped, timeout=None)
    waypoint_pub.publish(amcl_pose)

    r = rospy.Rate(1)

    for wp in waypoints_path:
        waypoint_pub.publish(wp)
        rospy.loginfo('contador do publisher: {}'.format(j))
        rospy.loginfo(wp)
        j += 1
        r.sleep()
    
    #Publicando a mensagem em branco que inicia o movimento
    #Futuramente, eh melhor dividir em dois servicos. Um publica os waypoints no Rviz, pra ser visualizado
    #em tempo real, e outro servico executa.
    execute_path_pub.publish()

    return MissionPathResponse('The robot {} is moving!'.format(robot_name))


if __name__ == "__main__":
    rospy.init_node('mission_path_server')
    s = rospy.Service('mission_path', MissionPath, transform_msg)
    
    rospy.loginfo('The mission_path service is ready to be used!')
    rospy.spin()

    #Nao esta publicando a primeira mensagem. 'mission_path_script.py' funciona corretamente.