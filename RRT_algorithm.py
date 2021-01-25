#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point, PoseArray
from turtlesim.msg import Pose
from people_msgs.msg import PositionMeasurementArray
from people_msgs.msg import PositionMeasurement
from math import pow, atan2, sqrt
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
import random
from math import sqrt
from utils import (Value, OrderedSet, PriorityQueue)

edges = {}
xs = 0.0
ys = 0.0
xp = 0.0
yp = 0.0
theta = 0.0
k_vel = 0.35
k_ang = 1.5
collide = False


# ///////////////// functions used in the program ////////////////////////

class turtlebot():

    def distance(q1, q2):
        x = (q1[0] - q2[0])
        x = x * x
        y = (q1[1] - q2[1])
        y = y * y
        dist = sqrt(x + y)
        return dist

    def GetId(node, nodes):
        id = nodes.index(node)
        return id

    def collision(x_pose, y_pose):
        obstacle_pos = [[1.5, 1.5], [3, 0], [5, 2], [3, 2.5], [1, 5], [2.5, 3.5], [3.5, 1.5], [4, 5], [0.5, 3], [4.5, -0.2], [1.5, 0]]
        collide = False
        for i in range(len(obstacle_pos)):
            obstacle = obstacle_pos[i]
            x0 = obstacle[0]
            y0 = obstacle[1]
            if sqrt(pow((x_pose - x0), 2) + pow((y_pose - y0), 2)) <= 0.65:
                collide = True
            else:
                pass
        return collide

    # ////////////////////////////////////////////////////////////////////////////////////////////

    start_point = [0, 0]  # turtlebot's starting position
    goal = [5, 3.5]  # goal position
    Edge = 0.35 # distance between each node
    nodes = []  # list containing all the nodes
    edges = {}
    nodes.append(start_point)
    # assigning the map boundaries
    x_start = -0.5
    x_end = 5.5
    y_start = -0.5
    y_end = 5.5
    goal_distance = distance(start_point, goal)

    # /////////////////// RRT PLANNING ///////////////////////////
    while goal_distance > 0.4: # new nodes are created until a node is formed within a distance of 0.2 from the goal
        print("creating nodes")
        x = random.uniform(x_start, x_end)
        x = round(x, 1)
        y = random.uniform(y_start, y_end)
        y = round(y, 1)
        rand_node = [x, y]  # A random node is selected
        least_dist = []

        if len(nodes) == 1:
            near_node = nodes[0]
        else:
            for i in range(len(nodes)):  # A for loop to calculate the distances between the random node and rest of the nodes
                node = nodes[i]
                Dist = distance(node, rand_node)
                least_dist.append(Dist)  # all the distances are appended to least_dist

            dist = least_dist[0]
            for i in range(len(least_dist)):  # The shortest distance is selected and the corresponding node.
                if least_dist[i] <= dist:
                    dist = least_dist[i]
                    K = i
                else:
                    pass
            near_node = nodes[K]  # the nearest node selected.

        D = distance(near_node, rand_node)

        if D < Edge:
            goal_distance = distance(near_node, goal)
        else:
            # Solving the equations of Line and Circle to find a point at a distance of 0.4 from the nearest node
            x0 = rand_node[0]
            x1 = near_node[0]
            y0 = rand_node[1]
            y1 = near_node[1]

            if x1 == x0:
                m = -1
            else:
                m = (y1 - y0)/(x1 - x0)
            c = y0 - (m * x0)

            A = (1 + (m*m))
            B = (-2*x1 + 2*m*c - 2*m*y1)
            C = (x1*x1) + c*c + (y1*y1) - 2*c*y1 - (Edge*Edge)
            if (B*B) < (4*A*C):
                pass
            else:
                X = sqrt((B*B) - 4*A*C)
                X = (-B + X)/(2*A)
                X = round(X, 2)
                Y = m*X + c
                Y = round(Y, 2)
                P1 = [X, Y]
                X1 = (-B - X)/(2*A)
                X1 = round(X1, 2)
                Y1 = m*X1 + c
                Y1 = round(Y1, 2)
                P2 = [X1, Y1]

                D = distance(rand_node, P1)
                D1 = distance(rand_node, P2)

            if D < D1:
                new_node = P1
            else:
                new_node = P2

            goal_distance = distance(new_node, goal)
        # /////////////////////////////Check the new node for collision //////////////////////////////////
        x_pose = new_node[0]
        y_pose = new_node[1]
        if distance(near_node, new_node) > (Edge + 1):
            pass
        else:
            if collision(x_pose, y_pose):
                pass
            else:
                nodes.append(new_node) # the new node is updated to the nodes list
                new_id = GetId(new_node, nodes)
                near_id = GetId(near_node, nodes)
                id_1 = new_id
                id_2 = near_id
                if id_1 in edges:
                    K = edges[id_1]
                    K.append(id_2)
                    edges.update({id_1: K})
                else:
                    edges.update({id_1: [id_2]})
                if id_2 in edges:
                    K1 = edges[id_2]
                    K1.append(id_1)
                    edges.update({id_2: K1})
                else:
                    edges.update({id_2: [id_1]})

    # ///////////////////////////// Path planning using A* ////////////////////////////
    print("finding shortest path")

    closed_set = OrderedSet()
    open_set = PriorityQueue(order=min, f=lambda v: v.f)
    g = 0
    h = distance(start_point, goal)
    f = g + h
    start_id = GetId(start_point, nodes)
    open_set.put(start_id, Value(f=f, g=g))
    parent = {}
    path = []
    g_check = []
    last_node = (len(nodes) - 1)
    nodes.append(goal)
    goal_id = GetId(goal, nodes)
    # AddEdge ///////////////////
    id_1 = last_node
    id_2 = goal_id
    if id_1 in edges:
        K = edges[id_1]
        K.append(id_2)
        edges.update({id_1: K})
    else:
        edges.update({id_1: [id_2]})
    if id_2 in edges:
        K1 = edges[id_2]
        K1.append(id_1)
        edges.update({id_2: K1})
    else:
        edges.update({id_2: [id_1]})
 #//////////////////////////////////////////
    while not len(open_set) == 0:
        pop_id, cost = open_set.pop()
        closed_set.add(pop_id)
        pop_g = cost.g
        if pop_id == goal_id:
            print("yes")
            while not pop_id == start_id:
                pop_id = parent[pop_id]
                path_node = nodes[pop_id]
                path.insert(0, path_node)
                # print(path)
        else:
            child = edges[pop_id]
            g_check.append(pop_id)
            for i in range(len(child)):
                child_id = child[i]
                if child_id not in g_check:
                    child_node = nodes[child_id]
                    parent_node = nodes[pop_id]
                    g = pop_g + distance(child_node, parent_node)
                    h = distance(child_node, goal)
                    parent.update({child_id: pop_id})
                    f = g + h
                    if child_id not in closed_set:
                        open_set.put(child_id, Value(f=f, g=g))
                    else:
                        pass
                else:
                    pass

    def odo_value(msg):
        global xs
        global ys
        global theta
        xs = msg.pose.pose.position.x
        ys = msg.pose.pose.position.y
        orient = msg.pose.pose.orientation
        (r, p, theta) = euler_from_quaternion ([orient.x, orient.y, orient.z, orient.w])
        theta = round(theta, 2)
        return xs, ys, theta

    global xp
    global yp
    global k_vel, k_ang, radius

    print(path)
    rospy.init_node('vel_closed', anonymous=True)
    velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    pose_subscriber = rospy.Subscriber('odom', Odometry, odo_value)
    pose = Pose()
    rate = rospy.Rate(10)
    vel_msg = Twist()
    path.remove(start_point)
    path.append(goal)
    # print(path)

    for i in range(len(path)):
        new_point = path[i]
        print(new_point)
        xp = new_point[0]
        yp = new_point[1]

        while sqrt(pow((xp - xs), 2) + pow((yp - ys), 2)) > 0.1:
            x_vel = (k_vel) * sqrt(pow((xp - xs), 2) + pow((yp - ys), 2))
            vel_msg.linear.x = min(0.3, x_vel)
            vel_msg.angular.z = (k_ang) * (atan2(yp - ys, xp - xs) - theta)
            velocity_publisher.publish(vel_msg)
            rate.sleep()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        x = turtlebot()

    except rospy.ROSInterruptException: pass
