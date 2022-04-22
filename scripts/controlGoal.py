#!/usr/bin/env python


import sys
import math
import rospy
import tf
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path 


class Turtlebot():

    def __init__(self):

        self.cmd_vel = rospy.Publisher('/cmd_vel_coll', Twist, queue_size=10)
        #self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/path', Path, self.callback)
        self.listener = tf.TransformListener()
        #Parametrizaciones
        self.max_linear = rospy.get_param('~max_linear_speed')
        self.min_linear = rospy.get_param('~min_linear_speed')
        self.max_angular = rospy.get_param('~max_angular_speed')
        self.tolerancia_angulo = rospy.get_param('~tolerancia_angulo')
        self.tolerancia_punto = rospy.get_param('~tolerancia_punto')
        self.tolerancia_destino = rospy.get_param('~tolerancia_destino')
        self.destino = False
        self.camino=None
        self.control=0

    def callback(self, path):
        rospy.loginfo("Camino recibido. Numero de puntos: %d", len(path.poses))
        self.poses = path.poses
        self.camino=path

    def linearVel(self, base_goal):
        x = base_goal.point.x
        y = base_goal.point.y
        distancia_modulo = math.sqrt(x*x + y*y)
        rospy.loginfo("Distancia: (%f)", distancia_modulo)
        vel = 0.2*distancia_modulo
        if(vel > self.max_linear):
            vel = self.max_linear
        elif(vel<self.min_linear):
            vel = self.min_linear
        return vel

    def angularVelParado(self, base_goal):
        x = base_goal.point.x
        y = base_goal.point.y
        if(y > 0):
            vel = self.max_angular
        else:
            vel = self.max_angular*-1
        return vel

    def angularVelMovimiento(self, base_goal):
        x = base_goal.point.x
        y = base_goal.point.y
        if(y > 0):
            vel = self.max_angular * math.atan2(y, x)
        else:
            vel = self.max_angular * math.atan2(y, x)
        return vel

    def command(self):
        try:
            goal = PointStamped()
            base_goal = PointStamped()

            goal.header.frame_id = "odom"  # rostopic echo /odom

            goal.header.stamp = rospy.Time()

            if(self.camino != None): #Si existe camino para seguir
                if(self.control < len(self.camino.poses)): #Si el numero de puntos recorridos (comienza en 0) es menor que el total de puntos
                    goal.point.x = self.camino.poses[self.control].pose.position.x
                    goal.point.y = self.camino.poses[self.control].pose.position.y
                    if(self.control == len(self.camino.poses)-1): #Si es el ultimo punto (punto destino)
                        self.destino = True
                        rospy.loginfo("Punto destino: X(%f) Y(%f)",goal.point.x,goal.point.y)
                    else:
                        rospy.loginfo("Siguiente punto: X(%f) Y(%f)",goal.point.x,goal.point.y)
                else: #Si se han recorrido todos los puntos
                    self.shutdown(self)


                base_goal = self.listener.transformPoint('base_footprint', goal)  # Dame el goal en mi sistema
                if(self.camino != None):
                    if(self.meta(base_goal)): #Si se ha llegado al punto
                        self.control += 1     #Aumentamos la variable de puntos recorridos
                if(abs(math.atan2(base_goal.point.y, base_goal.point.x)) > self.tolerancia_angulo): #Si la diferencia del angulo es mayor a max tolerancia angulo
                    angular = self.angularVelParado(base_goal)     #Girar con vel constante
                    #linear = 0 #Con velocidad linear 0
                    linear = 0.05  # Con velocidad linear 0.1
                else:      #Velocidad linear y angular relativa
                    angular = self.angularVelMovimiento(base_goal)
                    linear = self.linearVel(base_goal)
                rospy.loginfo("Velocidad linear: (%f)", linear)
                rospy.loginfo("Velocidad angular: (%f)", angular)
                self.publish(linear, angular)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("Problem TF")
            return


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
            if(self.destino == False): #Si no es el punto destino
                if(dist<self.tolerancia_punto): #La distancia debe ser menor a la tolerancia de los puntos del camino
                    return True
                else:
                    return False
            else: #Si es el punto destino
                if(dist<self.tolerancia_destino):#La distancia debe ser menor a la tolerancia del punto destino
                    return True
                else:
                    return False
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

        # TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
        r = rospy.Rate(10)

        # as long as you haven't ctrl + c keeping doing...
        while not rospy.is_shutdown():
            # publish the velocity
            robot.command()
            # wait for 0.1 seconds (10 HZ) and publish again
            r.sleep()

    except:
        rospy.loginfo("robotcontrol node terminated.")
