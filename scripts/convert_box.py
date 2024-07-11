#!/usr/bin/env python3

import json
import rospy
import numpy as np

import tf2_ros
import tf2_geometry_msgs

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry, OccupancyGrid

from occupancy_grid_python import OccupancyGridManager

class BoxChecker:
    def __init__(self):
        rospy.init_node('box_checker')
        self.destination_frame = rospy.get_param('~destination_frame', 'wcias_odom')
        self.minscore = rospy.get_param('~min_prob', 30)
        self.object = rospy.get_param('~object', 'person') # TODO transfom this in a list
        self.safe_value = rospy.get_param('~safe_value', 127)
        self.has_new_data = False
        self.makerlist = []

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def setup(self):
        self.setup_listeners()
        self.setup_publishers()

    def setup_listeners(self):
        self.map = OccupancyGridManager('/move_base/local_costmap/costmap', subscribe_to_updates=True)
        rospy.Subscriber('/yolo/dgb_bb_markers', MarkerArray, self.makerlist_callback)

    def setup_publishers(self):
        self.pose_pub = rospy.Publisher('/yolo_detected_objects_pose', PoseArray)
        self.dest_pub = rospy.Publisher('/yolo_detected_objects_destination', PoseArray)

    def makerlist_callback(self, msg):
        self.makerlist = msg.markers
        if np.size(msg.markers) > 0:
            self.makerlist_frame = msg.markers[0].header.frame_id
            self.has_new_data = True

    def transform_markers(self):

        transform = self.tf_buffer.lookup_transform(self.destination_frame, self.makerlist_frame,  rospy.Time(0), rospy.Duration(1.0))

        l = []
        for marker in self.makerlist:
            new_pose = tf2_geometry_msgs.do_transform_pose(marker, transform)
            marker.pose = new_pose.pose
            l.append(marker)

        self.l = l

        print(self.makerlist)


    def create_msg(self):
        msg = PoseArray()
        msg.header.frame_id = self.destination_frame
        msg.poses = self.convert_box_position()
        msg.header.stamp = rospy.Time.now()
        self.current_object = msg.poses 
        return msg

    def correct_poses(self):
        # Piglia il costo della posizione richiesta
        # Se la mappa li e' ok va bene, senno' spostare la posizione piu vicino alla sedia
        # Iterare fino a successo (al massimo sei arrivato)
        new_positions = []
        for obj in self.current_object:
            try:
                pos_x, pos_y = self.map.get_costmap_x_y(obj.position.x, obj.position.y)
                new_x, new_y , cost = self.map.get_closest_cell_under_cost(x=pos_x, y=pos_y, cost_threshold=self.safe_value, max_radius=40)
                world_x, world_y = self.map.get_world_x_y(new_x, new_y)

                obj.position.x = world_x
                obj.position.y = world_y
                    
                new_positions.append(obj)
            except:
                new_positions.append(obj)

        return new_positions

    def create_dest_msg(self):
        msg = PoseArray()
        msg.header.frame_id = self.destination_frame
        msg.poses = self.correct_poses()
        msg.header.stamp = rospy.Time.now()
        return msg

    def convert_box_position(self):
        poses = []
        # First convert the markers into the map frame
        self.transform_markers()
        # Now take the markers that are related to the object interested and add the relative position to the pose array
        # TODO: convert the position into a interesting thing in the map frame
        for marker in self.l:
            #print(marker.text.replace("'",'"'))
            #info['class_name']=marker.text # = # json.loads(marker.text.replace("'",'"'))
            if marker.text == self.object: # and info['score'] > self.minscore:
                if (marker.scale.x + marker.scale.y + marker.scale.z) > 0.1:
                    pose = Pose()
                    pose.position.x = marker.pose.position.x
                    pose.position.y = marker.pose.position.y
                    pose.position.z = marker.pose.position.z
                    pose.orientation = marker.pose.orientation
                    poses.append(pose)

        return poses

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.has_new_data:
                self.pose_pub.publish(self.create_msg())
                self.dest_pub.publish(self.create_dest_msg())
                self.has_new_data = False
            # print(self.map.get_world_x_y(0, 0))
            rate.sleep()

def main():
    checker = BoxChecker()
    checker.setup()
    checker.run()

if __name__ == '__main__':
    main()

