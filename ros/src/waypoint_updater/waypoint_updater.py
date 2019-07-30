#!/usr/bin/env python

import rospy
import math
import numpy as np
import tf
from scipy.spatial import KDTree
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32, Float32


LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        sub3 = rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        sub4 = rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)

        self.publisher = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.breaking_acceleration = None
        self.breaking_acceleration_limit = rospy.get_param('~deceleration_limit', -5)

        self.final_waypoints = None
        self.traffic_waypoint = -1
        self.stop = 0

        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()

    def pose_cb(self, msg): 
        self.current_pose = msg

    def waypoints_cb(self, waypoints): 
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg): 
        self.traffic_waypoint = msg.data

    def current_velocity_cb(self, msg): 
        self.current_velocity = msg.twist.linear.x

    def update(self):
        if not (hasattr(self, 'current_pose') and hasattr(self, 'base_waypoints')):
            return

        next_wp = self.get_next_waypoint()
        self.check_for_traffic_lights(next_wp)
        self.determine_final_waypoints(next_wp)
        self.publish_message()

    def get_next_waypoint(self):

        x = self.current_pose.pose.position.x
        y = self.current_pose.pose.position.y
        closest_wp = self.waypoint_tree.query([x,y], 1)[1]

        #Check if closest is ahead or behind vehicle
        closest_coord = self.waypoints_2d[closest_wp]
        prev_coord = self.waypoints_2d[closest_wp-1]

        #Equation for hyperplane through closest_coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect-prev_vect, pos_vect-cl_vect)

        if val > 0:
            closest_wp = (closest_wp + 1) % len(self.waypoints_2d)
        return closest_wp

    def check_for_traffic_lights(self, next_wp):
        if self.stop == 1:
            if self.traffic_waypoint == -1: 
                self.stop = 0
        elif self.stop == 0:
            if self.traffic_waypoint == -1:
                self.stop = 0
            else:
                deceleration = lambda x: abs(self.current_velocity**2 / (2*x))

                tl_distance = self.distance(self.base_waypoints.waypoints, next_wp, self.traffic_waypoint)
                min_distance = deceleration(self.breaking_acceleration_limit)

                if tl_distance > min_distance:
                    self.stop = 1
                    self.breaking_acceleration = deceleration(tl_distance)
                else: 
                    self.stop = 0

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt(pow((a.x - b.x), 2) + pow((a.y - b.y), 2) + pow((a.z - b.z), 2))
        for i in range(wp1, (wp2 + 1)):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def determine_final_waypoints(self, start_wp):
        self.final_waypoints = []

        if self.stop == 1:
            for i in range(start_wp, self.traffic_waypoint):
                j = i % len(self.base_waypoints.waypoints)
                wp_new = self.get_final_waypoint(i, 0)
                self.final_waypoints.append(wp_new)
                
            target_wp = len(self.final_waypoints)

            i_max = max(start_wp + LOOKAHEAD_WPS, self.traffic_waypoint + 1)
            for i in range(self.traffic_waypoint, i_max):
                wp_new = self.get_final_waypoint(i, 1)
                self.final_waypoints.append(wp_new)

            last = self.final_waypoints[target_wp]
            last.twist.twist.linear.x = 0.0

            for wp in self.final_waypoints[:target_wp][::-1]:
                x_diff = wp.pose.pose.position.x - last.pose.pose.position.x
                y_diff = wp.pose.pose.position.y - last.pose.pose.position.y
                z_diff = wp.pose.pose.position.z - last.pose.pose.position.z
                distance = math.sqrt(pow(x_diff, 2) + pow(y_diff, 2) + pow(z_diff, 2))
                velocity = math.sqrt(2 * self.breaking_acceleration * max(0.0, distance - 5))
                if velocity < 1.0: 
                    velocity = 0.0
                wp.twist.twist.linear.x = min(velocity, wp.twist.twist.linear.x)

        elif self.stop == 0:
            for i in range(start_wp, start_wp + LOOKAHEAD_WPS):
                j = i % len(self.base_waypoints.waypoints)
                wp_new = Waypoint()
                wp_new.pose.pose = self.base_waypoints.waypoints[j].pose.pose
                wp_new.twist.twist.linear.x = self.base_waypoints.waypoints[j].twist.twist.linear.x
                self.final_waypoints.append(wp_new)

    def get_final_waypoint(self, i, tl_passed):
        j = i % len(self.base_waypoints.waypoints)
        wp_new = Waypoint()
        wp_new.pose.pose = self.base_waypoints.waypoints[j].pose.pose
        if tl_passed:
            wp_new.twist.twist.linear.x  = 0.0
        else:
            wp_new.twist.twist.linear.x = self.base_waypoints.waypoints[j].twist.twist.linear.x
        
        return wp_new

    def publish_message(self):
        msg = Lane()
        msg.header.stamp = rospy.Time().now()
        msg.header.frame_id = 'world'
        msg.waypoints = self.final_waypoints[:LOOKAHEAD_WPS]
        self.publisher.publish(msg)

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
