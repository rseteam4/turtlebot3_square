#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class TurtlebotSquareAtOrigin(object):
    def __init__(self):
        rospy.init_node('turtlebot3_square_at_origin', anonymous=False)

        # Publisher for velocity commands
        self.pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber for odometry to get x, y, yaw
        self.current_x = None
        self.current_y = None
        self.current_yaw = None
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Wait for first odom message
        rospy.loginfo("Waiting for /odom messages...")
        while not rospy.is_shutdown() and (self.current_x is None or
                                           self.current_y is None or
                                           self.current_yaw is None):
            rospy.sleep(0.1)
        rospy.loginfo("Odometry received: x={:.3f}, y={:.3f}, yaw={:.3f}".format(
            self.current_x, self.current_y, self.current_yaw))

        # Move robot to origin (0,0,0) if needed
        self.move_to_origin()

        # Once at origin, perform square movement
        self.move_in_square()

    def odom_callback(self, msg):
        # Extract position
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Extract yaw (orientation around z)
        q = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_yaw = yaw

    def rotate_to_angle(self, target_angle, angular_speed=0.5):
        """
        Rotate in place until current_yaw is within 0.02 rad of target_angle.
        Uses fixed angular speed (±angular_speed).
        """
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            current = math.atan2(math.sin(self.current_yaw),
                                 math.cos(self.current_yaw))
            diff = target_angle - current
            diff = math.atan2(math.sin(diff), math.cos(diff))  # wrap to [-pi, pi]

            if abs(diff) < 0.02:
                break

            cmd = Twist()
            cmd.angular.z = angular_speed * (diff / abs(diff))
            self.pub_cmd.publish(cmd)
            rate.sleep()

        # Stop rotation
        stop = Twist()
        self.pub_cmd.publish(stop)
        rospy.sleep(0.2)

    def move_forward_straight(self, distance, speed=0.2, k_correction=1.0):
        """
        Move forward a given distance (meters) at speed (m/s), using yaw feedback
        to stay on the initial heading. k_correction applies to angular.z to correct yaw drift.
        """
        # Record starting pose
        start_x = self.current_x
        start_y = self.current_y
        start_yaw = self.current_yaw

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            # Compute traveled distance
            dx = self.current_x - start_x
            dy = self.current_y - start_y
            traveled = math.sqrt(dx*dx + dy*dy)
            if traveled >= distance - 0.02:
                break

            # Compute yaw error relative to start_yaw
            current = math.atan2(math.sin(self.current_yaw), math.cos(self.current_yaw))
            error_yaw = start_yaw - current
            error_yaw = math.atan2(math.sin(error_yaw), math.cos(error_yaw))

            cmd = Twist()
            cmd.linear.x = speed
            cmd.angular.z = k_correction * error_yaw
            self.pub_cmd.publish(cmd)
            rate.sleep()

        # Stop
        stop = Twist()
        self.pub_cmd.publish(stop)
        rospy.sleep(0.2)

    def move_to_origin(self):
        """
        1) Rotate to face the origin (angle = atan2(-y, -x))
        2) Drive forward until within 0.05 m of (0, 0), with yaw correction
        3) Rotate to yaw = 0
        """
        tol_pos = 0.05  # 5 cm
        tol_yaw = 0.02  # ~1.1°

        rospy.loginfo("Moving to origin (0,0,0)...")
        while not rospy.is_shutdown():
            x = self.current_x
            y = self.current_y
            dist = math.sqrt(x*x + y*y)
            if dist < tol_pos:
                break

            # Compute angle toward origin
            angle_to_goal = math.atan2(-y, -x)
            self.rotate_to_angle(angle_to_goal)
            self.move_forward_straight(dist, speed=0.2, k_correction=1.0)

        rospy.loginfo("Position near origin (x≈0, y≈0). Now orienting to yaw=0.")
        # Rotate to yaw = 0
        self.rotate_to_angle(0.0)
        rospy.loginfo("Reached (0,0,0).")

    def move_in_square(self):
        """
        Move in a 2m × 2m square, starting from (0,0,0).
        Each leg is 2 m forward (with yaw correction), followed by an exact 90° CCW rotation.
        """
        forward_distance = 2.0  # meters
        forward_speed = 0.2     # m/s
        angular_speed = 0.5     # rad/s

        rospy.loginfo("Starting square movement from origin.")
        for i in range(4):
            rospy.loginfo("Step {}/4: Moving forward {:.2f} m".format(i+1, forward_distance))
            self.move_forward_straight(forward_distance, speed=forward_speed, k_correction=1.0)

            rospy.loginfo("Step {}/4: Rotating 90 degrees".format(i+1))
            self.rotate_exactly_90(angular_speed)

        rospy.loginfo("Completed square. Shutting down.")
        self.pub_cmd.publish(Twist())
        rospy.signal_shutdown("Square path done")

    def rotate_exactly_90(self, angular_speed):
        """
        Rotate exactly π/2 radians counter-clockwise using odometry feedback.
        """
        start_yaw = self.current_yaw
        target = start_yaw + (math.pi / 2.0)
        target = math.atan2(math.sin(target), math.cos(target))

        rate = rospy.Rate(50)
        cmd = Twist()
        cmd.angular.z = angular_speed

        rospy.loginfo("Rotating from {:.3f} to {:.3f} rad".format(start_yaw, target))
        while not rospy.is_shutdown():
            current = self.current_yaw
            diff = target - current
            diff = math.atan2(math.sin(diff), math.cos(diff))
            if abs(diff) < 0.02:
                break
            self.pub_cmd.publish(cmd)
            rate.sleep()

        # Stop
        stop = Twist()
        self.pub_cmd.publish(stop)
        rospy.sleep(0.2)
        rospy.loginfo("Rotation done. Current yaw: {:.3f}".format(self.current_yaw))


if __name__ == '__main__':
    try:
        TurtlebotSquareAtOrigin()
    except rospy.ROSInterruptException:
        pass

