#!/usr/bin/env python3
from math import sqrt
from pathlib import Path as P
import sys
import numpy as np


import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, Point
from std_msgs.msg import Bool

from path_calculater import PathCalculator
from robot_controller import RobotController



class Navigation:
    """! Navigation node class.
    This class should server as a template to implement the path planning and 
    path follower components to move the turtlebot from position A to B.
    """

    def __init__(self, node_name='Navigation'):
        """! Class constructor.
        @param  None.
        @return An instance of the Navigation class.
        """

        #  log variables
        self.lab_4_pkg_path = P(__file__).parent.parent
        self.log_cleared = False

        
        # ROS related variables
        self.node_name = node_name
        self.rate = 10
        # Path planner/follower related variables
        self.map_name = "map"
        self.goal_pose = PoseStamped()
        self.ttbot_pose = PoseStamped()
        self.count = 0
        self.path_follower_path_control_num = 5
        self.path_complete = True

        # PID controller
        self.trans_p = 0.10
        self.trans_i = 0
        self.trans_d = 0
        self.ang_p = 1.5
        self.ang_i = 0
        self.ang_d = 0
        self.trans_finish_range = 0.5   # pixel
        self.angle_finish_range = 2     # degree

        

    def init_app(self):
        """! Node intialization.
        @param  None
        @return None.
        """
        # ROS node initilization

        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)
        # Subscribers
        rospy.Subscriber('/move_base_simple/goal', PoseStamped,
                         self.__goal_pose_cbk, queue_size=1)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped,
                         self.__ttbot_pose_cbk, queue_size=1)
        # Publishers
        self.path_pub = rospy.Publisher('global_plan', Path, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.log_clear_pub = rospy.Publisher('/clear_log', Bool, queue_size=1)
        self.robot_controller = RobotController(self.trans_p, self.trans_i, self.trans_d,
                                                 self.ang_p, self.ang_i, self.ang_d)
        
        self.robot_controller.set_trans_finish_range(self.trans_finish_range) # unit pixel
        
        
        self.robot_controller.set_rotation_finish_range(self.angle_finish_range / 180 * 3.1415926)


    def __goal_pose_cbk(self, data):
        """! Callback to catch the goal pose.
        @param  data    PoseStamped object from RVIZ.
        @return None.
        """
        current = data.pose.position
        self.goal_pose.pose.position.x = 95 - current.y * 10
        self.goal_pose.pose.position.y = 103 + current.x * 10
        self.path_complete = False
        
        target_orientation = data.pose.orientation
        x = target_orientation.x
        y = target_orientation.y
        z = target_orientation.z
        w = target_orientation.w
        roll, pitch, yaw = euler_from_quaternion([x, y, z, w])
        # rospy.loginfo(f'roll:{roll:.4f} pitch:{pitch:.4f} yaw:{yaw:.4f}')
        self.robot_controller.set_final_target(self.goal_pose.pose.position.x, self.goal_pose.pose.position.y, yaw + np.pi / 2)
        rospy.loginfo('goal_pose:{:.4f},{:.4f},{:.1f} deg '.format(
            self.goal_pose.pose.position.x, self.goal_pose.pose.position.y, yaw / 3.1415926 * 180))



    def __ttbot_pose_cbk(self, data):
        """! Callback to catch the position of the vehicle.
        @param  data    PoseWithCovarianceStamped object from amcl.
        @return None.
        """
        
        current = data.pose.pose.position
        self.ttbot_pose.pose.position.x = 95 - current.y * 10
        self.ttbot_pose.pose.position.y = 103 + current.x * 10
        orientation = data.pose.pose.orientation
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w
        if not self.log_cleared:
            f = open(self.lab_4_pkg_path / "log/robot.txt", 'w')
            f.close()
            self.log_cleared = True
        
        f = open(self.lab_4_pkg_path / "log/robot.txt", 'a')
        f.write(f'{self.ttbot_pose.pose.position.x},{self.ttbot_pose.pose.position.y}\n')
        f.close()
        roll, pitch, yaw = euler_from_quaternion([x, y, z, w])
        self.ttbot_pose.pose.orientation.w = yaw
        cov = data.pose.covariance
        
        current_state = self.robot_controller.get_current_state()
        rospy.loginfo('> ['+current_state.name+'] Pos:({:.2f},{:.2f})'.format(self.ttbot_pose.pose.position.x, self.ttbot_pose.pose.position.y)+f" Yaw:{self.ttbot_pose.pose.orientation.w:.2f}")


    def a_star_path_planner(self, start_pose, end_pose):
        """! A Start path planner.
        @param  start_pose    PoseStamped object containing the start of the path to be created.
        @param  end_pose      PoseStamped object containing the end of the path to be created.
        @return path          Path object containing the sequence of waypoints of the created path.
        """
        path = Path()
        rospy.loginfo('A* planner.\n> start:{:.2f},{:.2f},\n> end:{:.2f},{:.2f}'.format(
            start_pose.pose.position.x, start_pose.pose.position.y, end_pose.pose.position.x, end_pose.pose.position.y))
        rospy.loginfo(">---------- Calculation Initialized ---------<")
        pc = PathCalculator(self.map_name, start_pose.pose.position.x, start_pose.pose.position.y,
                            end_pose.pose.position.x, end_pose.pose.position.y)
        path = pc.get_path()
        rospy.loginfo(">---------- Calculation Finishied ---------<")
        self.log_clear_pub.publish(Bool(True))
        return path

    def get_distance(self, pos1, pos2):
        
        return sqrt((pos1.x-pos2.x) ** 2 + (pos1.y-pos2.y) ** 2)

    def get_current_goal(self, path, current_position):
        current = Point()
        current = current_position.pose.position
        path_positions_dict = {}
        path_positions = []
        i = 0
        for path_point_str in path:
            path_positions_dict.__setitem__(path_point_str, i)
            x, y = path_point_str.split(",")
            path_positions.append(Point(int(x),int(y),0))
            i += 1
        # Initiate Path Complete
        
        # sort 离当前位置最近
        path_positions.sort(key = lambda position: self.get_distance(position, current))

        nearestFewPositions = []
        num_of_pose = self.path_follower_path_control_num if self.path_follower_path_control_num < len(path_positions) else len(path_positions) - 1
        for i in range(num_of_pose):
            nearestFewPositions.append(path_positions[i])
        nearestFewPositions.sort(key=lambda pos: self.get_distance(self.goal_pose.pose.position, pos))

        
        nearest_position = nearestFewPositions[len(nearestFewPositions) - 1]
        nearestFewPositions.sort(key=lambda position: path_positions_dict[f"{position.x},{position.y}"])
        # for current_goal in nearestFewPositions:
        #     if path_positions_dict[f"{current_goal.x},{current_goal.y}"] - path_positions_dict[f"{nearest_position.x},{nearest_position.y}"] < num_of_pose - 1:
        #         return current_goal
        nearestFewPositions.reverse()
        current_goal = nearestFewPositions[0]
        return current_goal


    def path_follower(self, vehicle_pose, current_goal_position):
        """! Path follower.
        @param  vehicle_pose           PoseStamped object containing the current vehicle pose.
        @param  current_goal_pose      PoseStamped object containing the current target from the created path. This is different from the global target.
        @return path                   Path object containing the sequence of waypoints of the created path.
        """
        speed = 0
        heading = 0
        #rospy.loginfo(f"target: {current_goal_pose.pose.position.x},{current_goal_pose.pose.position.x}")
        self.robot_controller.set_target(current_goal_position)
        self.path_complete = self.robot_controller.path_completed()
        if self.path_complete:
            rospy.loginfo(f">---------- Path Complete ---------<")
        speed, heading = self.robot_controller.get_output(vehicle_pose.pose)
        return speed, heading

    def move_ttbot(self, linear, angular):
        """! Function to move turtlebot passing directly a heading angle and the speed.
        @param  speed     Desired yaw angle.
        @param  heading   Desired speed.
        @return path      object containing the sequence of waypoints of the created path.
        """
        cmd_vel = Twist()
        
        cmd_vel.linear.x = linear
        cmd_vel.angular.z = angular

        self.cmd_vel_pub.publish(cmd_vel)

    def run(self):
        """! Main loop of the node. You need to wait until a new pose is published, create a path and then
        drive the vehicle towards the final pose.
        @param none
        @return none
        """

        '''
            Main loop
        '''
        
        while not rospy.is_shutdown():
            a = 1
            if not self.path_complete:
                # 1. Create the path to follow
                path = self.a_star_path_planner(self.ttbot_pose, self.goal_pose)
                while not self.path_complete and not rospy.is_shutdown():
                    # 2. Loop through the path and move the robot
                    current_goal = self.get_current_goal(path, self.ttbot_pose)
                    speed, heading = self.path_follower(self.ttbot_pose, current_goal)
                    self.move_ttbot(speed, heading)
                    self.rate.sleep()
        f = open(self.lab_4_pkg_path / "log/robot.txt", 'w')
        f.close()
        f = open(self.lab_4_pkg_path / "log/path.txt", 'w')
        f.close()
        rospy.signal_shutdown("[{}] Finished Cleanly".format(self.node_name))



if __name__ == "__main__":
    nav = Navigation(node_name='Navigation')
    nav.init_app()
    try:
        nav.run()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
