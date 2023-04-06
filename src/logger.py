#!/usr/bin/env python3
import numpy as np
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from map_handler import MapHandler, MapProcessor
from pathlib import Path
import rospy
import sys

from std_msgs.msg import Bool

class Logger:

    def __init__(self, map_name, node_name):
        rospy.init_node(node_name, anonymous=True)
        
        rospy.Subscriber("/clear_log", Bool, callback = self.clear_log, queue_size=1)
        self.node_name = "logger"
        self.lab_4_pkg_path = Path(__file__).parent.parent
        self.map_name = map_name
        self.map_x_points = []
        self.map_y_points = []
        self.path_x_points = []
        self.path_y_points = []
        self.robot_x_points = []
        self.robot_y_points = []
        self.update_map_points()
        
        self.fig, self.ax = plt.subplots()
        self.fig.set_size_inches(10,10)
        self.ani = FuncAnimation(self.fig, self.refresh, interval = 100)
        self.begin()
        self.rate = rospy.Rate(1)
    
    def clear_log(self, data):

        if data.data:
            self.path_x_points = []
            self.path_y_points = []
            self.robot_x_points = []
            self.robot_y_points = []
             
    def begin(self):
        plt.show(block = True)

    def update_map_points(self):
        mp = MapProcessor(self.map_name)
        kr = mp.rect_kernel(3, 3)
        mp.inflate_map(kr, True)
        mp.get_graph_from_map(False)

        
        for node in mp.map_graph.g:
            a, b = node.split(",")
            self.map_x_points.append(a)
            self.map_y_points.append(b)



    def update_path_points(self):
        f = open(self.lab_4_pkg_path / "log/path.txt", "r")
        log_lines = f.readlines()
        f.close()

        for log_line in log_lines:

            x, y = log_line.split(",")
            x = float(x)
            y = float(y)

            self.path_x_points.append(x)
            self.path_y_points.append(y)

    def update_robot_points(self):
        f = open(self.lab_4_pkg_path / "log/robot.txt", "r")
        log_lines = f.readlines()
        f.close()
        for log_line in log_lines:
            
            x, y = log_line.split(",")
            x = float(x)
            y = float(y)

            self.robot_x_points.append(x)
            self.robot_y_points.append(y)

    def generate_visual(self):
        self.update_path_points()
        self.update_robot_points()
        
        self.fig.clear
        self.fig.canvas.flush_events()
    
    def refresh(self, i):
        
        plt.cla()
        
        
        

        self.update_path_points()
        self.update_robot_points()
        
        self.map_plot = self.ax.scatter(self.map_x_points, self.map_y_points, s=25)
        self.path_plot = self.ax.scatter(self.path_x_points, self.path_y_points, s = 50)
        self.robot_plot = self.ax.scatter(self.robot_x_points, self.robot_y_points,s = 80)
        # plt.tight_layout()
        self.ax.set_xlim([66, 125])
        self.ax.set_ylim([70, 135])
        self.ax.set_xticks([70, 80, 90, 100, 110, 120])
        self.ax.set_yticks([70, 80, 90, 100, 110, 120, 130])
        self.ax.tick_params(labelsize=15)
        self.ax.set_title("Path Animation", fontsize = 30)
        self.ax.legend((self.map_plot, self.path_plot, self.robot_plot), ("Map", "Desired Path", "Robot Path"), fontsize="20", loc ="upper right")

        if rospy.is_shutdown(): plt.close()
        
    def run(self):

        while not rospy.is_shutdown():
            self.rate.sleep()
        plt.close()
        rospy.signal_shutdown("[{}] Finished Cleanly".format(self.node_name))
    
        
if __name__ == "__main__":

    
    try:
        logger = Logger(map_name = "map", node_name='Navigation')
        logger.run()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
