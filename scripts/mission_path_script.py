#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from std_msgs.msg import Empty
from tf.transformations import quaternion_from_euler
from math import radians

def Quaternion_Transform(orientacao):
    yaw = radians(orientacao)
    quaternion = quaternion_from_euler(0, 0, yaw)
    quat_msg = Quaternion()
    quat_msg.x = quaternion[0]
    quat_msg.y = quaternion[1]
    quat_msg.z = quaternion[2]
    quat_msg.w = quaternion[3]
    return quat_msg

def Mission_Path():
    wp_count = 0

    robot_name = raw_input("Digite o namespace do robo que receberá a missão: ")
    #Criando os publishers necessários
    waypoint_pub = rospy.Publisher('{}/wp_topic'.format(robot_name), PoseWithCovarianceStamped, queue_size=10)
    execute_path_pub = rospy.Publisher('{}/path_ready'.format(robot_name), Empty, queue_size=10)

    while wp_count >= 0:
        print("Digite a {}ª coordenada e a orientação desejada.".format(wp_count + 1))
        x = float(raw_input("X: "))
        y = float(raw_input("Y: "))
        orientacao = float(raw_input("Orientação (Em graus, em relação ao eixo X): "))

        #Criar Mensagem
        wp_msg = PoseWithCovarianceStamped()
        #Preenchendo o Header
        wp_msg.header.stamp = rospy.Time.now()
        wp_msg.header.frame_id = "map"
        #Preenchendo a Pose Msg
        wp_msg.pose.pose.position.x = x
        wp_msg.pose.pose.position.y = y
        wp_msg.pose.pose.position.z = 0
        #Preenchendo o Quaternion
        quaternion = Quaternion_Transform(orientacao)
        wp_msg.pose.pose.orientation = quaternion
        #Publicando mensagem no tópico que guarda os waypoints
        waypoint_pub.publish(wp_msg)

        continuar = raw_input("Deseja adicionar mais uma coordenada? Digite 'y' para SIM e 'n' para NÃO: ")
        if continuar == 'y':
            wp_count += 1
            continue
        else:
            break
        
    #Publicando mensagem que fará o robô executar o caminho
    execute_path_pub.publish()
    print('O robô {} executará o trajeto determinado.'.format(robot_name))


if __name__ == "__main__":
    rospy.init_node('mission_path')
    Mission_Path()
