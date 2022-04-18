#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import tf.transformations
import math
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point

topic = 'visualization_marker_array'
rospy.init_node('listener', anonymous=True)
publisher = rospy.Publisher(topic, MarkerArray, queue_size= 10)

j1 = 0
j2 = 0
j3 = 0
j4 = 0

def callback(js):
    print "JS=", js
    global j1, j2, j3, j4
    j1 = js.position[3]
    j2 = js.position[0]
    j3 = js.position[1]
    j4 = js.position[2]

callback(rospy.wait_for_message("joint_states", JointState))

"""
Ок. теперь можем посчитать положения.
будем считать положения концов всех 3х цилиндров последнего линка (4)
всего у нас 6 торцов.
Каждый цилиндр определён как длина и радиус. При этом изначально цилиндр
имеет начало в середине длины и направлен вдоль оси Z.

координаты торца l4_1: (0, 0, l4_len/2) и (0, 0, -l4_len/2)
координаты торца l4_2: (0, 0, l4_len/4) и (0, 0, -l4_len/4)
координаты торца l4_3: (0, 0, l4_len) и (0, 0, -l4_len)

""" 
l4_len = 0.3
l3_len = 0.35
l2_len = 0.23
l1_len = 0.165

l4_1_l = np.array([0, 0, -l4_len/2])
l4_1_u = np.array([0, 0, l4_len/2])

l4_2_l = np.array([0, 0, -l4_len/4])
l4_2_u = np.array([0, 0, l4_len/4])

l4_3_l = np.array([0, 0, -l4_len])
l4_3_u = np.array([0, 0, l4_len])
"""
вычислим положения этих точек в системе координат joint_link003_link004
Для это произведём преобразования описанные origin в теге visual.
При этом все точки объекта смещаются и поворачиваются.
Для каждого мы повернёмся на rpy а затем сместимся на соответствующий shift
"""
l4_1_shift = np.array([l4_len/2, 0, 0])
l4_2_shift = np.array([l4_len/4, 0, l4_len/3])
l4_3_shift = np.array([l4_len, 0, -l4_len/3])

rpy_trans_1 = tf.transformations.euler_matrix(math.pi, math.pi/2, 0, 'rzyx')[:3, :3]
rpy_trans_2 = tf.transformations.euler_matrix(math.pi, math.pi/2, 0, 'rzyx')[:3, :3]
rpy_trans_3 = tf.transformations.euler_matrix(math.pi, math.pi/2, 0, 'rzyx')[:3, :3]

l4_1_l = rpy_trans_1.dot(l4_1_l)
l4_1_u = rpy_trans_1.dot(l4_1_u)

l4_2_l = rpy_trans_2.dot(l4_2_l)
l4_2_u = rpy_trans_2.dot(l4_2_u)

l4_3_l = rpy_trans_3.dot(l4_3_l)
l4_3_u = rpy_trans_3.dot(l4_3_u)

l4_1_l += l4_1_shift
l4_1_u += l4_1_shift

l4_2_l += l4_2_shift
l4_2_u += l4_2_shift

l4_3_l += l4_3_shift
l4_3_u += l4_3_shift

"""Отметим эти точки и дадим насладиться всем этим"""
markerArray = MarkerArray()
for i, j in enumerate(((l4_1_l, (1,1,0)), (l4_1_u, (1,0,1)),\
     (l4_2_l, (1,0,0)), (l4_2_u, (1,1,0)),\
        (l4_3_l, (0,1,0)), (l4_3_u, (0,1,1)) )):
    marker = Marker()
    marker.header.frame_id = "link_004"
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = 0.02
    marker.scale.y = 0.02
    marker.scale.z = 0.02
    marker.color.r =j[1][0]
    marker.color.g =j[1][1]
    marker.color.b =j[1][2]
    marker.color.a = 1
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = j[0][0]
    marker.pose.position.y = j[0][1]
    marker.pose.position.z = j[0][2]
    marker.id = i
    markerArray.markers.append(marker)

publisher.publish(markerArray)

raw_input("Press Enter to continue...")

markerArrayDel = MarkerArray()
for i in range(6):
    marker = Marker()
    marker.header.frame_id = "link_004"
    marker.action = marker.DELETE
    marker.id = i
    markerArrayDel.markers.append(marker)

publisher.publish(markerArrayDel) 


"""ok. now let's delete these markers and calculate to world frame coordinates"""
""" Мы повернулись на j4 угол. Найдём матрицу преобразования """
yaxis = (0, 1, 0)
Ry = tf.transformations.rotation_matrix(j4, yaxis)[:3, :3]
"""преобразуем координаты точек повёрнутой оси в координатах "оригинальной" системы координат"""
l4_1_l = Ry.dot(l4_1_l)
l4_1_u = Ry.dot(l4_1_u)
l4_2_l = Ry.dot(l4_2_l)
l4_2_u = Ry.dot(l4_2_u)
l4_3_l = Ry.dot(l4_3_l)
l4_3_u = Ry.dot(l4_3_u)

""" повернём координатную систему в соответствии с ориентацией joint_link003_link004 """
rpy_trans_j3_j4 = tf.transformations.euler_matrix(0, math.pi, 0, 'rzyx')[:3,:3]

l4_1_l = rpy_trans_j3_j4.dot(l4_1_l)
l4_1_u = rpy_trans_j3_j4.dot(l4_1_u)
l4_2_l = rpy_trans_j3_j4.dot(l4_2_l)
l4_2_u = rpy_trans_j3_j4.dot(l4_2_u)
l4_3_l = rpy_trans_j3_j4.dot(l4_3_l)
l4_3_u = rpy_trans_j3_j4.dot(l4_3_u)

l3_shift = np.array([0, l3_len, 0])
l4_1_l += l3_shift
l4_1_u += l3_shift
l4_2_l += l3_shift
l4_2_u += l3_shift
l4_3_l += l3_shift
l4_3_u += l3_shift

"""Отметим линиями из начала link3"""
markerArray = MarkerArray()
pointZero = Point(x=0, y=0, z=0)
for i, j in enumerate(((l4_1_l, (1,1,0)), (l4_1_u, (1,0,1)),\
     (l4_2_l, (1,0,0)), (l4_2_u, (1,1,0)),\
        (l4_3_l, (0,1,0)), (l4_3_u, (0,1,1)) )):
    marker = Marker()
    marker.header.frame_id = "link_003"
    marker.type = marker.LINE_LIST
    marker.action = marker.ADD
    marker.scale.x = 0.02
    marker.scale.y = 0.02
    marker.scale.z = 0.02
    marker.color.r =j[1][0]
    marker.color.g =j[1][1]
    marker.color.b =j[1][2]
    marker.color.a = 1
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.points.append(pointZero)
    marker.points.append(Point(x = j[0][0], y=j[0][1], z=j[0][2]))
    marker.id = 10 + i
    markerArray.markers.append(marker)

publisher.publish(markerArray)

markerArrayDel2 = MarkerArray()
for i in range(6):
    marker = Marker()
    marker.header.frame_id = "link_003"
    marker.action = marker.DELETE
    marker.id = 10 + i
    markerArrayDel2.markers.append(marker)

publisher.publish(markerArrayDel2) 


""" а теперь без остановок летим до world системы координат"""
"""joint_link002_link003 вращается вокруг X """
xaxis = (1, 0, 0)
Rx_j3 = tf.transformations.rotation_matrix(j3, xaxis)[:3, :3]

l4_1_l = Rx_j3.dot(l4_1_l)
l4_1_u = Rx_j3.dot(l4_1_u)
l4_2_l = Rx_j3.dot(l4_2_l)
l4_2_u = Rx_j3.dot(l4_2_u)
l4_3_l = Rx_j3.dot(l4_3_l)
l4_3_u = Rx_j3.dot(l4_3_u)

rpy_trans_j2_j3 = tf.transformations.euler_matrix(2.0943951023931953, 0, -1.0471975511965976, 'rzyx')[:3,:3]

l4_1_l = rpy_trans_j2_j3.dot(l4_1_l)
l4_1_u = rpy_trans_j2_j3.dot(l4_1_u)
l4_2_l = rpy_trans_j2_j3.dot(l4_2_l)
l4_2_u = rpy_trans_j2_j3.dot(l4_2_u)
l4_3_l = rpy_trans_j2_j3.dot(l4_3_l)
l4_3_u = rpy_trans_j2_j3.dot(l4_3_u)

l2_shift = np.array([l2_len * (-5.00000000e-01), l2_len*8.66025404e-01, 0])
l4_1_l += l2_shift
l4_1_u += l2_shift
l4_2_l += l2_shift
l4_2_u += l2_shift
l4_3_l += l2_shift
l4_3_u += l2_shift
""" j2 вращается вокруг какой-то лабуды"""
j2_rot_vec = (0.4330127, 0.25, 0.8660254)
R2 = tf.transformations.rotation_matrix(j2, j2_rot_vec)[:3, :3]
l4_1_l = R2.dot(l4_1_l)
l4_1_u = R2.dot(l4_1_u)
l4_2_l = R2.dot(l4_2_l)
l4_2_u = R2.dot(l4_2_u)
l4_3_l = R2.dot(l4_3_l)
l4_3_u = R2.dot(l4_3_u)
rpy_trans_j1_j2 = tf.transformations.euler_matrix( -0.8570719478501307, 0.848062078981481, -0.7137243789447655, 'rzyx')[:3, :3]

l4_1_l = rpy_trans_j1_j2.dot(l4_1_l)
l4_1_u = rpy_trans_j1_j2.dot(l4_1_u)
l4_2_l = rpy_trans_j1_j2.dot(l4_2_l)
l4_2_u = rpy_trans_j1_j2.dot(l4_2_u)
l4_3_l = rpy_trans_j1_j2.dot(l4_3_l)
l4_3_u = rpy_trans_j1_j2.dot(l4_3_u)

l1_shift = np.array([l1_len, 0, 0])
l4_1_l += l1_shift
l4_1_u += l1_shift
l4_2_l += l1_shift
l4_2_u += l1_shift
l4_3_l += l1_shift
l4_3_u += l1_shift


"""и преобразование world -> link1 имеет только поворот вокруг y
остальные смещения и повороты - нулевые"""
Ry_j1 = tf.transformations.rotation_matrix(j1, yaxis)[:3, :3]
l4_1_l = Ry_j1.dot(l4_1_l)
l4_1_u = Ry_j1.dot(l4_1_u)
l4_2_l = Ry_j1.dot(l4_2_l)
l4_2_u = Ry_j1.dot(l4_2_u)
l4_3_l = Ry_j1.dot(l4_3_l)
l4_3_u = Ry_j1.dot(l4_3_u)

""" отрисуем линии из world системы координат"""
markerArrayFinal = MarkerArray()
pointZero = Point(x=0, y=0, z=0)
for i, j in enumerate(((l4_1_l, (1,1,0)), (l4_1_u, (1,0,1)),\
     (l4_2_l, (1,0,0)), (l4_2_u, (1,1,0)),\
        (l4_3_l, (0,1,0)), (l4_3_u, (0,1,1)) )):
    marker = Marker()
    marker.header.frame_id = "world"
    marker.type = marker.LINE_LIST
    marker.action = marker.ADD
    marker.scale.x = 0.02
    marker.scale.y = 0.02
    marker.scale.z = 0.02
    marker.color.r =j[1][0]
    marker.color.g =j[1][1]
    marker.color.b =j[1][2]
    marker.color.a = 1
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.points.append(pointZero)
    marker.points.append(Point(x = j[0][0], y=j[0][1], z=j[0][2]))
    marker.id = 10 + i
    markerArrayFinal.markers.append(marker)

publisher.publish(markerArrayFinal)


