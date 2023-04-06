#!/usr/bin/env python3
import numpy as np
import rospy

from map_handler import Queue, MapHandler
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from pathlib import Path as P

class AStar():
    def __init__(self, in_tree):
        self.lab_4_pkg_path = P(__file__).parent.parent
        self.in_tree = in_tree
        self.q = Queue()
        self.dist = {name: np.Inf for name, node in in_tree.g.items()}
        self.h = {name: 0 for name, node in in_tree.g.items()}
        self.visited = {name: False for name, node in in_tree.g.items()}

        for name, node in in_tree.g.items():
            start = tuple(map(int, name.split(',')))
            end = tuple(map(int, self.in_tree.end.split(',')))
            self.h[name] = np.sqrt((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2)

        self.via = {name: 0 for name, node in in_tree.g.items()}


    def __get_f_score(self, node):
        return self.h[node.name]

    def solve(self, sn, en):
        i = 0
        self.dist[sn.name] = 0
        self.q.push(sn)
        f = open(self.lab_4_pkg_path / "log/log.txt", "w")
        f.close()
        f = open(self.lab_4_pkg_path / "log/log.txt", "w")
        f.write("("+sn.name+") --> ("+en.name+")\n")
        while len(self.q) > 0 or not rospy.is_shutdown():
            self.q.sort(key=self.__get_f_score)
            u = self.q.pop()
            
            if not self.visited[u.name]:
                self.visited[u.name] = True
                new_dist = self.dist[u.name] + 1
                rospy.loginfo("   Solving -> (" + u.name + ")")
                
                f.write(u.name + "\n")
                
                if u.name == en.name:
                    f.close()
                    break
                for i in range(len(u.children)):
                    c = u.children[i]
                    if not self.visited[c.name]:
                        self.q.push(c)
                        if new_dist < self.dist[c.name]:
                            self.dist[c.name] = new_dist
                            self.via[c.name] = u.name

    def reconstruct_path(self, sn, en):
        start_key = sn.name
        end_key = en.name
        dist = self.dist[end_key]
        u = end_key
        path = [u]
        while u != start_key:
            u = self.via[u]
            path.append(u)
        path.reverse()
        return path, dist


class PathCalculator:

    def __init__(self, map_name, start_pos_x, start_pos_y, end_pose_x, end_pose_y):
        self.lab_4_pkg_path = P(__file__).parent.parent
        self.map_name = map_name
        self.root = f"{start_pos_x:.0f},{start_pos_y:.0f}"
        self.end = f"{end_pose_x:.0f},{end_pose_y:.0f}"
        
    def get_path(self):
        
        mh = MapHandler(self.map_name)
        mp = mh.get_map()
        mp.get_graph_from_map()
        mp.map_graph.root = self.root
        mp.map_graph.end = self.end
        
        rospy.loginfo(">  Map Created")

        as_maze = AStar(mp.map_graph)

        rospy.loginfo(">  Solving in Progress")

        as_maze.solve(mp.map_graph.g[mp.map_graph.root], mp.map_graph.g[mp.map_graph.end])

        path_as, dist_as = as_maze.reconstruct_path(mp.map_graph.g[mp.map_graph.root], mp.map_graph.g[mp.map_graph.end])

        f = open(self.lab_4_pkg_path / "log/path.txt", 'w')
        

        path = Path()

        for pose_string in path_as:
            f.write(pose_string + '\n')
            x_string, y_string = pose_string.split(",")
            pose = PoseStamped()
            pose.pose.position.x = int(x_string)
            pose.pose.position.y = int(y_string)
            path.poses.append(pose)

        f.close()

        return path_as
