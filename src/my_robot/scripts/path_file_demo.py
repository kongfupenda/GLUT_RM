#! /usr/bin/env python

"""
    Python 版 HelloWorld

"""
import rospy
import open3d as o3d

index = 101 #列表0开始
data_read =[]
data_write = []

if __name__ == "__main__":
    rospy.init_node("path_file_demo")
    rospy.loginfo("Hello path_file_demo!!!!")

    f_read = open('/home/pangyuhui/autonomous_exploration_development_environment/src/local_planner/paths/paths.txt', 'r')
    #'w': 写模式。如果文件不存在，则创建文件。如果文件已存在，则清空文件并写入新内容
    f_wirte = open('/home/pangyuhui/autonomous_exploration_development_environment/src/local_planner/paths/pathList_wirte.txt', 'w')
    data_read = f_read.readlines()
    for i in range(342):
        index = index+301
        try:
            data_write = data_read[index]
        except IndexError:
            print("The index is out of range.")
        f_wirte.write(data_write)
        print(data_write)

    # with open('data.txt', 'w') as f:
    #     data = 'some data to be written to the file'
    #     f.write(data)