#!/usr/bin/env python3
from geometry_msgs.msg import Pose, Point
from math import sqrt, atan2, pi
from enum import Enum
from pathlib import Path

class State(Enum):
    REST = "At Rest"
    HEADING = "Heading"
    IN_PATH = "In Path"
    ORIENTATION_CALI = "Cali"

class RobotController:
    def __init__(self, p_trans, i_trans, d_trans, p_rotation, i_rotation, d_rotation):
        self.lab_4_pkg_path = Path(__file__).parent.parent
        
        self.p_trans = p_trans
        self.i_trans = i_trans
        self.d_trans = d_trans

        self.p_rotation = p_rotation
        self.i_rotation = i_rotation
        self.d_rotation = d_rotation

        self.target = Pose()
        self.current = Pose()
        self.final_pos_x = 0
        self.final_pos_y = 0
        
        self.last_trans_e = 0
        self.total_trans_e = 0
        self.last_rotation_e = 0
        self.total_rotation_e = 0

        self.trans_finish_range = 0
        self.rotation_finish_range = 0
        self.heading_finish_flag = True
        self.trans_finish_flag = True
        self.rotation_finish_flag = True
        self.state = State.REST

        
    
    def set_final_target(self, target_x, target_y, target_angle):
        f = open(self.lab_4_pkg_path / "log/robot.txt", "w")
        f.close()
        self.final_pos_x = target_x
        self.final_pos_y = target_y
        self.target.orientation.z = target_angle
        self.trans_finish_flag = False
        self.rotation_finish_flag = False
        self.heading_finish_flag = False

    def set_trans_finish_range(self, finish_range):
        self.trans_finish_range = finish_range
    
    def set_rotation_finish_range(self, finish_range):
        self.rotation_finish_range = finish_range

    def is_trans_error_in_range(self, e):
        return True if abs(e) < self.trans_finish_range else False

    def is_rotation_error_in_range(self, e):
        return True if abs(e) < self.rotation_finish_range else False

    def trans_finished(self):
        return self.trans_finish_flag

    def rotation_finished(self):
        return self.rotation_finish_flag

    def clear_e_memory(self):
        self.last_trans_e = 0
        self.total_trans_e = 0
        self.last_angle_e = 0
        self.total_angle_e = 0

    def set_target(self, target):
        final_orientaion = self.target.orientation.z
        self.target.position = target
        self.target.orientation.z = final_orientaion
        
        
        self.clear_e_memory()

    def get_current_state(self):
        
        if not self.heading_finish_flag:
            return State.HEADING
        if not self.trans_finish_flag:
            return State.IN_PATH
        elif not self.rotation_finish_flag:
            return State.ORIENTATION_CALI
        return State.REST
    
    def path_completed(self):
        return True if self.get_current_state() == State.REST else False

    def get_output(self, current):
        self.current = current

        robot_x = self.current.position.x
        robot_y = self.current.position.y
        robot_w = self.current.orientation.w + pi/2

        target_x = self.target.position.x
        target_y = self.target.position.y
        target_w = atan2(target_y - robot_y, target_x - robot_x)
        
        self.target_pose_w = self.target.orientation.w

        trans_e = sqrt((robot_x - target_x) ** 2 + (robot_y - target_y) ** 2)
        rotation_e = robot_w - target_w
        

        rotation_e = rotation_e % (2 * pi)
        target_distance = 0
        if rotation_e > pi : rotation_e -= 2 * pi

        self.total_trans_e += trans_e
        self.total_rotation_e += rotation_e

        current_state = self.get_current_state()
        if current_state == State.REST:
            linear = 0
            angular = 0

        elif current_state == State.HEADING:
            linear = 0
            
            if not self.is_rotation_error_in_range(rotation_e):
                angular = self.p_rotation * rotation_e + self.i_rotation * self.total_rotation_e + self.d_rotation * (rotation_e - self.last_rotation_e)
            else:
                self.heading_finish_flag = True
                angular = 0

        elif current_state == State.IN_PATH:
            # if not self.is_trans_error_in_range(trans_e):
            #     linear = self.p_trans * trans_e + self.i_trans * self.total_trans_e + self.d_trans * (trans_e - self.last_trans_e)
            #     if linear > 0.5:
            #         linear = 0.5
                
            # else:
            #     linear = 0
                

            linear = self.p_trans * trans_e + self.i_trans * self.total_trans_e + self.d_trans * (trans_e - self.last_trans_e)
            if linear > 0.5:
                linear = 0.5
    
                
            target_distance = sqrt( (robot_y-self.final_pos_y) ** 2 + (robot_x-self.final_pos_x) ** 2)
            if self.is_trans_error_in_range(target_distance):
                linear = 0
                self.trans_finish_flag = True
            

            if not self.is_rotation_error_in_range(rotation_e):
                angular = self.p_rotation * rotation_e + self.i_rotation * self.total_rotation_e + self.d_rotation * (rotation_e - self.last_rotation_e)
            else:
                angular = 0

        elif current_state == State.ORIENTATION_CALI:

            linear = 0
            rotation_e = robot_w - self.target.orientation.z
            rotation_e = rotation_e % (2 * pi)
            if rotation_e > pi : rotation_e -= 2 * pi
            if not self.is_rotation_error_in_range(rotation_e):
                angular = self.p_rotation * rotation_e + self.i_rotation * self.total_rotation_e + self.d_rotation * (rotation_e - self.last_rotation_e)
            else:
                self.rotation_finish_flag = True
                angular = 0

        

        f = open(self.lab_4_pkg_path / "log/log.txt", 'a')
        f.write(f"target_dist:{target_distance:.4f} trans:{trans_e:.4f} rot:{rotation_e:.4f} State:" + current_state.name + "\n")
        f.close()
        return linear, -angular