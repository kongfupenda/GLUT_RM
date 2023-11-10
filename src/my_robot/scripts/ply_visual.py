#! /usr/bin/env python

"""
    Python 版 HelloWorld

"""
import rospy
import open3d as o3d

if __name__ == "__main__":
    rospy.init_node("Hello")
    rospy.loginfo("Hello World!!!!")
    pcd = o3d.io.read_point_cloud('/home/pangyuhui/autonomous_exploration_development_environment/src/local_planner/paths/paths.ply')
    o3d.visualization.draw_geometries([pcd])
