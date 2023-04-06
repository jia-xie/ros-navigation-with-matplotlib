#!/usr/bin/env python3
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Twist



class PoseWithDistance:
    def __init__(self, pose) -> None:
        self.pose = pose

    def get_square_distance(self, pose1):
        return (self.pose.pose.position.x-pose1.pose.position.y) ** 2 + (self.pose.pose.position.y-pose1.pose.position.y) ** 2

if __name__ == '__main__':
    # pc = PathCalculator(93, 114, 107, 107)
    # pc.get_path()
    path_as = ['93,114',
            '94,114',
            '95,114',
            '96,114',
            '97,114',
            '98,113',
            '99,112',
            '100,112',
            '101,112',
            '102,112',
            '103,111',
            '104,110',
            '105,109',
            '106,108',
            '107,107']
    
    path = Path()

    for pose_string in path_as:
        x_string, y_string = pose_string.split(",")
        pose = PoseStamped()
        pose.pose.position.x = int(x_string)
        pose.pose.position.y = int(y_string)
        path.poses.append(pose)

    # print(path)
    
    current = PoseStamped()
    current.pose.position.x = 93
    current.pose.position.y = 114

    target = PoseStamped()
    current.pose.position.x =107
    current.pose.position.y = 107

    poses = path.poses

    poses.sort(key=lambda pose: 
                (pose.pose.position.x-current.pose.position.y) ** 2 
            + (pose.pose.position.y-current.pose.position.y) ** 2)
    poses.reverse()
    # print(poses[0])

    nearestFewPoses = []
    for i in range(3):
        nearestFewPoses.append(poses[i])
    nearestFewPoses.sort(key=lambda pose:
                (pose.pose.position.x-target.pose.position.y) ** 2 
            + (pose.pose.position.y-target.pose.position.y) ** 2)
    nearestFewPoses.reverse()
    print(nearestFewPoses[0])
    current.pose
    current.pose.position.x
    