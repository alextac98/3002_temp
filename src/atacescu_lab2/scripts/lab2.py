import rospy, tf, copy, math

from geometry_msgs.msg import Twist, Pose, PoseStamped
from tf.transformations import euler_from_quaternion
import numpy as np
from std_msgs.msg import String

class Robot:

    def __init__(self):

        """
            This constructor sets up class variables and pubs/subs
        """

        self._current = Pose() # initlize correctly
        self._odom_list = tf.TransformListener()
        rospy.Timer(rospy.Duration(.1), self.timerCallback)
        self._vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('nav2pose', Pose, self.navToPose, queue_size=1) # handle nav goal events

        # based on wheel track from http://emanual.robotis.com/docs/en/platform/turtlebot3/specifications/
        self.diameter = 0.066  # in meters
        self.wheelBase = 0.16  # in meters


    def navToPose(self,goal):
        """
            This is a callback function. It should exract data from goal, drive in a striaght line to reach the goal and
            then spin to match the goal orientation.

            Run driveStraight and spinWheels

        """

        self._odom_list.waitForTransform('YOUR_STRING_HERE', 'YOUR_STRING_HERE', rospy.Time(0), rospy.Duration(1.0))
        transGoal = self._odom_list.transformPose('YOUR_STRING_HERE', goal) # transform the nav goal from the global coordinate system to the robot's coordinate system

    def executeTrajectory(self):
      """
        See lab manual for the dance the robot has to excute

        write the directions in a state machine
      """

    def driveStraight(self, speed, distance):
        """
            This method should populate a ??? message type and publish it to ??? in order to move the robot

            Run v*changeintime
        """

        origin = copy.deepcopy(self._current)  # hint: use this

    def rotate(self, angle):
        """
            This method calls spin wheels to publish the commands to the wheels

            run 2*v*t/l
        """

        speed = 0.2  # rad/s

        origin = copy.deepcopy(self._current)

        q = [origin.orientation.x,
             origin.orientation.y,
             origin.orientation.z,
             origin.orientation.w]  # quaternion nonsense

        (roll, pitch, yaw) = euler_from_quaternion(q)

        angle

    def spinWheels(self, v_left, v_right, time):
        """
           This method should use differential drive kinematics to compute V and omega (linear x = V, angular z = omega).
           It should then create a [twist] message type, and publish it to [_vel_pub] in order to move the robot

           using changes in time, add fwd kin formulas

        """
        twist = Twist()

        # Set relevant values
        twist.linear.x = (v_right + v_left) / 2
        twist.angular.z = (v_right + v_left) / self.wheelBase

        # Set irrelevant values
        twist.linear.y = 0; twist.linear.z  = 0
        twist.angular.x = 0; twist.angular.y = 0

        driveStartTime = rospy.Time.now().secs

        while rospy.Time.now().secs < (driveStartTime + time):
            self._vel_pub.publish(twist)

    def timerCallback(self,evprent):
        """
            This is a callback that runs every 0.1s.
            Updates this instance of Robot's internal position variable (self._current)
        """
        # wait for and get the transform between two frames
        self._odom_list.waitForTransform('YOUR_STRING_HERE', 'YOUR_STRING_HERE', rospy.Time(0), rospy.Duration(1.0))
        (position, orientation) = self._odom_list.lookupTransform('YOUR_STRING_HERE', 'YOUR_STRING_HERE', rospy.Time(0))
        # save the current position and orientation
        self._current.position.x = position[0]
        self._current.position.y = position[1]
        self._current.orientation.x = orientation[0]
        self._current.orientation.y = orientation[1]
        self._current.orientation.z = orientation[2]
        self._current.orientation.w = orientation[3]

        # create a quaternion
        q = [self._current.orientation.x,
             self._current.orientation.y,
             self._current.orientation.z,
             self._current.orientation.w]

        # convert the quaternion to roll pitch yaw
        (roll, pitch, yaw) = euler_from_quaternion(q)

    # helper functions
    def planTraj(self, b, t):
        """
            Bonus Question:  compute the coefs for a cubic polynomial (hint:  you did this in 3001)
        """


if __name__ == '__main__':

    rospy.init_node('drive_base')
    turtle = Robot()

    # test function calls here

    while  not rospy.is_shutdown():
        turtle.spinWheels(turtle, 0.1, 0.1, 5)
