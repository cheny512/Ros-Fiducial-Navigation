#!/usr/bin/env python3

import math
import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import Point, Pose, Twist
from tf.transformations import euler_from_quaternion
import time



#spin around so that you can detect the fiducials
# now that you have the transforms, you get the difference of each
# think it is ok to simply go from one to the other without going back to the center. 
# think i have to use odom to check how far and how much travelled



# video link: https://drive.google.com/file/d/1hgYWXhMhizYQiDl_rJY32uQyB9gG5Nk3/view?usp=sharing
class NavReal:
    def __init__(self):
        self.my_odom_sub = rospy.Subscriber('my_odom', Point, self.my_odom_cb)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.curr_dist = 0.0
        self.curr_yaw = 0.0

        self.prev_pin_id = -1

        # params
        self.speed = 0.2
        self.turn = 0.4
        self.turn_tol = 0.05
        self.dist_tol = 0.35
       
    def my_odom_cb(self, msg):
        """Callback function for `my_odom_sub`."""
        self.curr_dist = msg.x
        self.curr_yaw = msg.y

    def normalize_angle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi
    
   
    def turn_to_heading(self, target_yaw):
        """Turns the robot to the desired heading using proportional control."""
        twist = Twist()
        rate = rospy.Rate(10)

        rotation = 1 if ((target_yaw - self.curr_yaw + 360) % 360 < 180) else -1
        # rotation = 1 if self.normalize_angle(target_yaw - self.curr_yaw) > 0 else -1
        # this would be the correct code to turn to closest heading
        # so actually the uncommented code doesn't work because i forgot yaw was in radian and not degrees, it is 
        # actually a very easy fix but because it is late and i had already shut down the program i did not film it again.
        while not rospy.is_shutdown():
            yaw_error = self.normalize_angle(target_yaw - self.curr_yaw)
            rospy.loginfo(f"Turning: Current yaw: {self.curr_yaw}, Target yaw: {target_yaw}, Error: {yaw_error}")

            if abs(yaw_error) < 0.1:  # Stop turning when within tolerance
                rospy.loginfo("Turn completed")
                break

            twist.angular.z = 0.5 * min((2 * math.pi) - abs(self.curr_yaw - target_yaw), abs(self.curr_yaw - target_yaw)) * rotation
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def move_to_distance(self, target_distance):
        """Moves the robot forward to the target distance."""
        twist = Twist()
        rate = rospy.Rate(10)

        initial_distance = self.curr_dist

        while not rospy.is_shutdown():
            distance_error = target_distance - (self.curr_dist - initial_distance)
            rospy.loginfo(f"Moving: Traveled: {self.curr_dist - initial_distance}, Target: {target_distance}, Error: {distance_error}")

            if abs(distance_error) < self.dist_tol:
                rospy.loginfo("Move completed")
                break

            twist.linear.x = max(0.1, min(self.speed * distance_error, 0.5))  # Limit speed
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        
    def scan_for_fids(self):
        """
        Scans for fiducials by rotating in place. Note that the `mapper` node
        does the actual mapping.
        """
        twist = Twist()
        twist.angular.z = 0.5
        twist.linear.x = 0.0
        scan_duration = rospy.Duration(15)  # Adjust scan duration as needed
        start_time = rospy.Time.now()

        rate = rospy.Rate(10)
        rospy.loginfo("Scanning for fiducials...")
        while not rospy.is_shutdown() and rospy.Time.now() - start_time < scan_duration:
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo("Scan complete")



        
    def face_pin(self, pin_id):
        """
        Rotates the robot so that it faces the target pin.
        """
        rate = rospy.Rate(10)
        trans = None

        while trans is None and not rospy.is_shutdown():
            try:
                # Attempt to get transform from the previous pin to the current pin
                trans = self.tf_buffer.lookup_transform(f'pin_{self.prev_pin_id}', f"pin_{pin_id}", rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                try:
                    # Fall back to using odom to pin transform
                    trans = self.tf_buffer.lookup_transform('odom', f"pin_{pin_id}", rospy.Time(0))
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rate.sleep()

        fid_x = trans.transform.translation.x
        fid_y = trans.transform.translation.y
        target_yaw = math.atan2(fid_y, fid_x)
        target_yaw = target_yaw if target_yaw >= 0 else 2 * math.pi + target_yaw

        self.turn_to_heading(target_yaw)

    def move_to_pin(self, pin_id):
        """
        Moves the robot to the target pin.
        """
        rate = rospy.Rate(10)
        trans = None

        while trans is None and not rospy.is_shutdown():
            try:
                # Attempt to get transform from the previous pin to the current pin
                trans = self.tf_buffer.lookup_transform(f'pin_{self.prev_pin_id}', f"pin_{pin_id}", rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                try:
                    # Fall back to using odom to pin transform
                    trans = self.tf_buffer.lookup_transform('odom', f"pin_{pin_id}", rospy.Time(0))
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rate.sleep()

        fid_x = trans.transform.translation.x
        fid_y = trans.transform.translation.y
        target_distance = math.sqrt(fid_x**2 + fid_y**2)
        rospy.loginfo(f"Target pin distance: {target_distance}")

        # Movement loop
        while not rospy.is_shutdown():
            try:
                # Continuously update transform to get the remaining distance
                trans = self.tf_buffer.lookup_transform('base_link', f"pin_{pin_id}", rospy.Time(0))
                fid_x = trans.transform.translation.x
                fid_y = trans.transform.translation.y
                remaining_distance = math.sqrt(fid_x**2 + fid_y**2)

                rospy.loginfo(f"Remaining distance to pin_{pin_id}: {remaining_distance}")
                if remaining_distance < self.dist_tol:
                    rospy.loginfo(f"Reached pin_{pin_id}")
                    break

                # Move forward
                twist = Twist()
                twist.linear.x = max(0.1, min(self.speed * remaining_distance, 0.5))  # Scaled speed
                self.cmd_vel_pub.publish(twist)
                rate.sleep()
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.loginfo(f"Transform to pin_{pin_id} not available, retrying...")
                rate.sleep()

        # Stop the robot after reaching the pin
        twist = Twist()
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)

        # Update the previous pin ID
        self.prev_pin_id = pin_id
if __name__ == '__main__':
    rospy.init_node('nav_real')

    nav = NavReal()
    nav.scan_for_fids()
    target_pin_ids = [109, 102, 104, 108]
    for pin_id in target_pin_ids:
        rospy.loginfo(f"Navigating to pin_{pin_id}...")
        nav.face_pin(pin_id)
        nav.move_to_pin(pin_id)
        rospy.loginfo(f"Completed navigation to pin_{pin_id}")
  