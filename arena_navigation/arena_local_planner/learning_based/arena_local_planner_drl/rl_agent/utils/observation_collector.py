#! /usr/bin/env python
from typing import Tuple

from datetime import datetime

from numpy.core.numeric import normalize_axis_tuple
import rospy
import random
import numpy as np

import time # for debuging

# observation msgs
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D,PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from arena_plan_msgs.msg import RobotState,RobotStateStamped

# for converting ROS messages to dictionaries
from rospy_message_converter import message_converter

# services
from flatland_msgs.srv import StepWorld,StepWorldRequest

import json
import rosbag

# message filter
import message_filters

# for transformations
from tf.transformations import *

from gym import spaces
import numpy as np



class ObservationCollector():
    def __init__(self,num_lidar_beams:int,lidar_range:float):
        """ a class to collect and merge observations

        Args:
            num_lidar_beams (int): [description]
            lidar_range (float): [description]
        """
        # define observation_space
        self.observation_space = ObservationCollector._stack_spaces((
            spaces.Box(low=0, high=lidar_range, shape=(num_lidar_beams,), dtype=np.float32),
            spaces.Box(low=0, high=10, shape=(1,), dtype=np.float32) ,
            spaces.Box(low=-np.pi, high=np.pi, shape=(1,), dtype=np.float32) 
        ))

        # flag of new sensor info
        self._flag_all_received=False

        self._scan = LaserScan()
        self._robot_pose = Pose2D()
        self._robot_vel = Twist()
        self._subgoal =  Pose2D()        
        self._cmd_vel = Twist()

        # message_filter subscriber: laserscan, robot_pose
        self._scan_sub = message_filters.Subscriber("scan", LaserScan)
        self._robot_state_sub = message_filters.Subscriber('robot_state', RobotStateStamped)

        # command velocity subscriber
        self._cmd_vel_sub = message_filters.Subscriber('cmd_vel', Twist)
        
        # message_filters.TimeSynchronizer: call callback only when all sensor info are ready
        self.ts = message_filters.ApproximateTimeSynchronizer([self._scan_sub, self._robot_state_sub, self._cmd_vel_sub], 100,slop=0.05,allow_headerless=True)
        self.ts.registerCallback(self.callback_observation_received)
        
        # topic subscriber: subgoal
        #TODO should we synchoronize it with other topics
        #self._subgoal_sub = message_filters.Subscriber('plan_manager/subgoal', PoseStamped) # self._subgoal_sub = rospy.Subscriber("subgoal", PoseStamped, self.callback_subgoal)
        self._subgoal_sub = message_filters.Subscriber('subgoal', PoseStamped)
        self._subgoal_sub.registerCallback(self.callback_subgoal)
        
        # service clients
        self._is_train_mode = rospy.get_param("train_mode")
        if self._is_train_mode:
            self._service_name_step='step_world'
            self._sim_step_client = rospy.ServiceProxy(self._service_name_step, StepWorld)


    
    def get_observation_space(self):
        return self.observation_space

    def get_observations(self):
        # reset flag 
        self._flag_all_received=False
        if self._is_train_mode: 
        # sim a step forward until all sensor msg uptodate
            i=0
            while(self._flag_all_received==False):
                self.call_service_takeSimStep()
                i+=1
        # rospy.logdebug(f"Current observation takes {i} steps for Synchronization")
        #print(f"Current observation takes {i} steps for Synchronization")
        scan=self._scan.ranges.astype(np.float32)
        rho, theta = ObservationCollector._get_goal_pose_in_robot_frame(self._subgoal,self._robot_pose)

        merged_obs = np.hstack([scan, np.array([rho,theta])])
        obs_dict = {}
        obs_dict["laser_scan"] = scan
        obs_dict['goal_in_robot_frame'] = [rho,theta]
        obs_dict['robot_pose'] = self._robot_pose
        obs_dict['cmd_vel'] = self._cmd_vel
        return merged_obs, obs_dict
    
    @staticmethod
    def _get_goal_pose_in_robot_frame(goal_pos:Pose2D,robot_pos:Pose2D):
         y_relative = goal_pos.y - robot_pos.y
         x_relative = goal_pos.x - robot_pos.x
         rho =  (x_relative**2+y_relative**2)**0.5
         theta = (np.arctan2(y_relative,x_relative)-robot_pos.theta+4*np.pi)%(2*np.pi)-np.pi
         return rho,theta


    def call_service_takeSimStep(self):
        request=StepWorldRequest()
        try:
            response=self._sim_step_client(request)
            rospy.logdebug("step service=",response)
        except rospy.ServiceException as e:
            rospy.logdebug("step Service call failed: %s"%e)

    def callback_subgoal(self,msg_Subgoal):
        self._subgoal=self.process_subgoal_msg(msg_Subgoal)
        return
        
    def callback_observation_received(self,msg_LaserScan,msg_RobotStateStamped, msg_cmd_vel):
        self._cmd_vel = msg_cmd_vel
        
        # process sensor msg
        self._scan=self.process_scan_msg(msg_LaserScan)
        self._robot_pose,self._robot_vel=self.process_robot_state_msg(msg_RobotStateStamped)

        # ask subgoal service
        #TODO create and call subgoal service to ensure the observation collector has the latest subgoal
        # reason: subgoal is published rarely, may default to (0,0,0)
        #self._subgoal=self.call_service_askForSubgoal()
        self._flag_all_received=True
        
    def process_scan_msg(self, msg_LaserScan):
        # remove_nans_from_scan
        scan = np.array(msg_LaserScan.ranges)
        scan[np.isnan(scan)] = msg_LaserScan.range_max
        msg_LaserScan.ranges = scan
        return msg_LaserScan
    
    def process_robot_state_msg(self,msg_RobotStateStamped):
        state=msg_RobotStateStamped.state
        pose3d=state.pose
        twist=state.twist
        return self.pose3D_to_pose2D(pose3d), twist
        
    def process_pose_msg(self,msg_PoseWithCovarianceStamped):
        # remove Covariance
        pose_with_cov=msg_PoseWithCovarianceStamped.pose
        pose=pose_with_cov.pose
        return self.pose3D_to_pose2D(pose)
    
    def process_subgoal_msg(self,msg_Subgoal):
        pose2d=self.pose3D_to_pose2D(msg_Subgoal.pose)
        return pose2d

    @staticmethod
    def pose3D_to_pose2D(pose3d):
        pose2d=Pose2D()
        pose2d.x=pose3d.position.x
        pose2d.y=pose3d.position.y
        quaternion=(pose3d.orientation.x,pose3d.orientation.y,pose3d.orientation.z,pose3d.orientation.w)
        euler = euler_from_quaternion(quaternion)
        yaw = euler[2]
        pose2d.theta=yaw
        return pose2d
    @staticmethod
    def _stack_spaces(ss:Tuple[spaces.Box]):
        low = []
        high = []
        for space in ss:
            low.extend(space.low.tolist())
            high.extend(space.high.tolist())
        return spaces.Box(np.array(low).flatten(),np.array(high).flatten())
    
    def _save_rollouts_as_json(self, cmd_vels, subgoals, robot_poses, laser_scans):
        timesteps = len(cmd_vels)
        data_object = {}
        # iterate over all timesteps in the arrays, convert ROS messages to JSON formatted strings, and add to a dictionary
        for i in range(timesteps):
            data_object[i] = {'cmd_vel': message_converter.convert_ros_message_to_dictionary(cmd_vels[i], True), 'subgoal': subgoals[i], 'robot_pose': message_converter.convert_ros_message_to_dictionary(robot_poses[i], True), 'laser_scan': laser_scans[i].tolist()}
        
        date_str = datetime.now().strftime('%Y%m%d_%H-%M')
        path = f'/home/michael/rollout_{date_str}.json'
        
        with open(path, "w", encoding='utf-8') as target:
            json.dump(data_object, target, ensure_ascii=False, indent=4)


if __name__ == '__main__':
    
    rospy.init_node('states', anonymous=True)
    print("start")
    state_collector=ObservationCollector(360,10)
    
    i=0
    r=rospy.Rate(100)

    cmd_vels = []
    subgoals = []
    robot_poses = []
    laser_scans = []

    while(i<=20):
        i = i + 1
        # obs contains (merged_obs, obs_dict) each consisting of laser scan ranges and goal in robot frame (rho, theta), (dict also has robot_pose, cmd_vel)
        obs = state_collector.get_observations()
        cmd_vels.append(obs[1]['cmd_vel'])
        subgoals.append(obs[1]['goal_in_robot_frame'])
        robot_poses.append(obs[1]['robot_pose'])
        laser_scans.append(obs[1]['laser_scan'])
        
        time.sleep(0.001)
    
    state_collector._save_rollouts_as_json(cmd_vels, subgoals, robot_poses, laser_scans)
