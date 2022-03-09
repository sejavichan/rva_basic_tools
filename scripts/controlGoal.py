#!/usr/bin/env python


# A very basic TurtleBot script that moves TurtleBot forward indefinitely. Press CTRL + C to stop.  To run:
# On TurtleBot:
# roslaunch turtlebot_bringup minimal.launch
# On work station:
# python goforward.py

import sys
import math
import rospy
import tf
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path 


class Turtlebot():

    def __init__(self):

        # Create a publisher which can "talk" to TurtleBot and tell it to move
        # Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/path', Path,self.callback)
        self.listener = tf.TransformListener()
        self.camino=None
        self.control=0

    def callback(self, path):
        rospy.loginfo("Camino recibido. numero de puntos: %d", len(path.poses))
        self.poses = path.poses
        self.camino=path

    def linearVel(self, base_goal):
        x = base_goal.point.x
        y = base_goal.point.y
        distancia_modulo = math.sqrt(x*x + y*y)
        rospy.loginfo("Distancia: (%f)", distancia_modulo)
        if(distancia_modulo > 0.005):
            vel = 0.2*distancia_modulo
            if(vel > 1):
                #vel = 0.8
                vel = rospy.get_param('~max_linear_speed')
            elif(vel<0.2):
                vel = rospy.get_param('~min_linear_speed')
        else:
            vel = 0
        return vel

    def angularVel(self, base_goal):
        x = base_goal.point.x
        y = base_goal.point.y
        if(y > 0):
            #signo = math.atan2(y, x)
            vel = 0.3
        else:
            #signo = math.atan2(y, x)+math.pi
            vel = -0.3
        return vel

    def command(self, gx, gy):
        goal = PointStamped()
        base_goal = PointStamped()

        goal.header.frame_id = "odom"  # rostopic echo /odom

        goal.header.stamp = rospy.Time()

        goal.point.x = gx
        goal.point.y = gy
        goal.point.z = 0.0

        if(self.camino != None):
            if(self.control < len(self.camino.poses)):
                goal.point.x = self.camino.poses[self.control].pose.position.x
                goal.point.y = self.camino.poses[self.control].pose.position.y
                rospy.loginfo("Siguiente punto: X(%f) Y(%f)",goal.point.x,goal.point.y)

        try:
            base_goal = self.listener.transformPoint('base_footprint', goal)  # Dame el goal en mi sistema
            if(self.camino != None):
                if(self.meta(base_goal)):
                    self.control += 1
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("Problem TF")
            return

        # TODO: put the control law here
        #linear = self.linearVel(base_goal)
        if(abs(math.atan2(base_goal.point.y, base_goal.point.x)) > 0.1):
            angular = self.angularVel(base_goal)
            linear = 0
        else:
            angular=0
            linear = self.linearVel(base_goal)
        rospy.loginfo("Velocidad linear: (%f)", linear)
        rospy.loginfo("Velocidad angular: (%f)", angular)

        if(angular == 0 and linear == 0):
            self.shutdown(self)

        self.publish(linear, angular)

    def publish(self, lin_vel, ang_vel):
        # Twist is a datatype for velocity
        move_cmd = Twist()
        # let's go forward at 0.2 m/s
        move_cmd.linear.x = lin_vel
        # let's turn at 0 radians/s
        move_cmd.angular.z = ang_vel

        self.cmd_vel.publish(move_cmd)

    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)

    def meta(self, base_goal):
        try:
            x = base_goal.point.x
            y = base_goal.point.y
            dist = math.sqrt(x*x + y*y)
            if(dist<0.1):
                return 1
            else:
                return 0
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("Problem TF")
            return


if __name__ == '__main__':
    try:
        # initiliaze
        rospy.init_node('robotcontrol', anonymous=False)

        # tell user how to stop TurtleBot
        rospy.loginfo("To stop TurtleBot CTRL + C")

        robot = Turtlebot()
        # What function to call when you ctrl + c
        rospy.on_shutdown(robot.shutdown)

        #goalx = float(sys.argv[1])
        #goaly = float(sys.argv[2])
        goalx = rospy.get_param('~goalx')
        goaly = rospy.get_param('~goaly')

        #goalx = 0
        #goaly = 0

        print(goalx)
        print(goaly)


        # TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
        r = rospy.Rate(10)

        # as long as you haven't ctrl + c keeping doing...
        while not rospy.is_shutdown():
            rospy.loginfo("Loop")
            # publish the velocity
            robot.command(goalx, goaly)
            # wait for 0.1 seconds (10 HZ) and publish again
            r.sleep()

    except:
        rospy.loginfo("robotcontrol node terminated.")
