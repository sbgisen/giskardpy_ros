from typing import Optional

import rospy
from giskardpy.data_types.data_types import Derivatives
from giskardpy.model.collision_avoidance_config import CollisionAvoidanceConfig
from giskardpy.model.collision_world_syncer import CollisionCheckerLib
from giskardpy.model.world_config import WorldWithDiffDriveRobot
from giskardpy_ros.configs.giskard import RobotInterfaceConfig


class WorldWithSOARConfig(WorldWithDiffDriveRobot):
    def __init__(self, map_name: str = 'map', localization_joint_name: str = 'localization',
                 odom_link_name: str = 'odom', drive_joint_name: str = 'brumbrum'):
        super().__init__(urdf=rospy.get_param('robot_description'),
                         map_name=map_name,
                         localization_joint_name=localization_joint_name,
                         odom_link_name=odom_link_name,
                         drive_joint_name=drive_joint_name)

    def setup(self):
        super().setup()
        self.set_joint_limits(limit_map={Derivatives.velocity: 1,
                                         Derivatives.jerk: None},
                              joint_name='head_pan_joint')
        self.set_joint_limits(limit_map={Derivatives.velocity: 3.5,
                                         Derivatives.jerk: None},
                              joint_name='head_tilt_joint')

        self.set_joint_limits(limit_map={Derivatives.velocity: 0.15,
                                         Derivatives.jerk: None},
                              joint_name='torso_joint')


class SOARStandaloneInterface(RobotInterfaceConfig):
    drive_joint_name: str

    def __init__(self, drive_joint_name: str):
        self.drive_joint_name = drive_joint_name

    def setup(self):
        self.register_controlled_joints([
            'torso_joint',
            'head_pan_joint',
            'head_tilt_joint',
            'arm_joint1',
            'arm_joint2',
            'arm_joint3',
            'arm_joint4',
            'arm_joint5',
            'arm_joint6',
            'arm_joint7',
            self.drive_joint_name,
        ])


class SOARJointTrajServerMujocoInterface(RobotInterfaceConfig):
    map_name: str
    localization_joint_name: str
    odom_link_name: str
    drive_joint_name: str

    def __init__(self,
                 map_name: str = 'map',
                 localization_joint_name: str = 'localization',
                 odom_link_name: str = 'odom_combined',
                 drive_joint_name: str = 'brumbrum'):
        self.map_name = map_name
        self.localization_joint_name = localization_joint_name
        self.odom_link_name = odom_link_name
        self.drive_joint_name = drive_joint_name

    def setup(self):
        self.sync_6dof_joint_with_tf_frame(joint_name=self.localization_joint_name,
                                           tf_parent_frame=self.map_name,
                                           tf_child_frame=self.odom_link_name)
        self.sync_joint_state_topic('/soar/joint_states')
        self.sync_odometry_topic('/soar/odom', self.drive_joint_name)
        self.add_follow_joint_trajectory_server(
            namespace='/soar/torso_trajectory_position_controller')
        self.add_follow_joint_trajectory_server(
            namespace='/soar/arm_trajectory_position_controller')
        self.add_follow_joint_trajectory_server(
            namespace='/soar/head_trajectory_position_controller')
        self.add_base_cmd_velocity(cmd_vel_topic='/soar/diff_drive_controller/cmd_vel',
                                   track_only_velocity=True,
                                   joint_name=self.drive_joint_name)


class SOARVelocityMujocoInterface(RobotInterfaceConfig):
    map_name: str
    localization_joint_name: str
    odom_link_name: str
    drive_joint_name: str

    def __init__(self,
                 map_name: str = 'map',
                 localization_joint_name: str = 'localization',
                 odom_link_name: str = 'odom_combined',
                 drive_joint_name: str = 'brumbrum'):
        self.map_name = map_name
        self.localization_joint_name = localization_joint_name
        self.odom_link_name = odom_link_name
        self.drive_joint_name = drive_joint_name

    def setup(self):
        self.sync_6dof_joint_with_tf_frame(joint_name=self.localization_joint_name,
                                           tf_parent_frame=self.map_name,
                                           tf_child_frame=self.odom_link_name)
        self.sync_joint_state_topic('/soar/joint_states')
        self.sync_odometry_topic('/soar/odom', self.drive_joint_name)
        self.add_joint_velocity_controller(namespaces=[
            'soar/torso_trajectory_position_controller',
            'soar/arm_trajectory_position_controller',
        ])

        self.add_base_cmd_velocity(cmd_vel_topic='/soar/diff_drive_controller/cmd_vel',
                                   joint_name=self.drive_joint_name)


class SOARVelocityIAIInterface(RobotInterfaceConfig):
    map_name: str
    localization_joint_name: str
    odom_link_name: str
    drive_joint_name: str

    def __init__(self,
                 map_name: str = 'map',
                 localization_joint_name: str = 'localization',
                 odom_link_name: str = 'odom_combined',
                 drive_joint_name: str = 'brumbrum'):
        self.map_name = map_name
        self.localization_joint_name = localization_joint_name
        self.odom_link_name = odom_link_name
        self.drive_joint_name = drive_joint_name

    def setup(self):
        self.sync_6dof_joint_with_tf_frame(joint_name=self.localization_joint_name,
                                           tf_parent_frame=self.map_name,
                                           tf_child_frame=self.odom_link_name)
        self.sync_joint_state_topic('/soar/joint_states')
        self.sync_odometry_topic('/soar/odom', self.drive_joint_name,
                                 sync_in_control_loop=False)
        self.add_joint_velocity_group_controller(namespace='soar/torso_trajectory_position_controller')
        self.add_joint_velocity_group_controller(namespace='soar/arm_trajectory_position_controller')

        self.add_base_cmd_velocity(cmd_vel_topic='/base_controller/command',
                                   joint_name=self.drive_joint_name)


class SOARCollisionAvoidance(CollisionAvoidanceConfig):
    def __init__(self, drive_joint_name: str = 'brumbrum',
                 collision_checker: CollisionCheckerLib = CollisionCheckerLib.bpb):
        super().__init__(collision_checker=collision_checker)
        self.drive_joint_name = drive_joint_name

    def setup(self):
        self.load_self_collision_matrix('package://soar_moveit_config/config/soar.srdf')
        self.set_default_external_collision_avoidance(soft_threshold=0.1,
                                                      hard_threshold=0.0)
        # for joint_name in ['r_wrist_roll_joint', 'l_wrist_roll_joint']:
        #     self.overwrite_external_collision_avoidance(joint_name,
        #                                                 number_of_repeller=4,
        #                                                 soft_threshold=0.05,
        #                                                 hard_threshold=0.0,
        #                                                 max_velocity=0.2)
        # for joint_name in ['r_wrist_flex_joint', 'l_wrist_flex_joint']:
        #     self.overwrite_external_collision_avoidance(joint_name,
        #                                                 number_of_repeller=2,
        #                                                 soft_threshold=0.05,
        #                                                 hard_threshold=0.0,
        #                                                 max_velocity=0.2)
        # for joint_name in ['r_elbow_flex_joint', 'l_elbow_flex_joint']:
        #     self.overwrite_external_collision_avoidance(joint_name,
        #                                                 soft_threshold=0.05,
        #                                                 hard_threshold=0.0)
        # for joint_name in ['r_forearm_roll_joint', 'l_forearm_roll_joint']:
        #     self.overwrite_external_collision_avoidance(joint_name,
        #                                                 soft_threshold=0.025,
        #                                                 hard_threshold=0.0)
        # self.fix_joints_for_collision_avoidance([
        #     'r_gripper_l_finger_joint',
        #     'l_gripper_l_finger_joint'
        # ])
        self.overwrite_external_collision_avoidance(self.drive_joint_name,
                                                    number_of_repeller=2,
                                                    soft_threshold=0.2,
                                                    hard_threshold=0.1)


class SOARJointTrajServerIAIInterface(RobotInterfaceConfig):
    map_name: str
    localization_joint_name: str
    odom_link_name: str
    drive_joint_name: str

    def __init__(self,
                 map_name: str = 'map',
                 localization_joint_name: str = 'localization',
                 odom_link_name: str = 'odom',
                 drive_joint_name: str = 'brumbrum'):
        self.map_name = map_name
        self.localization_joint_name = localization_joint_name
        self.odom_link_name = odom_link_name
        self.drive_joint_name = drive_joint_name

    def setup(self):
        self.sync_6dof_joint_with_tf_frame(joint_name=self.localization_joint_name,
                                           tf_parent_frame=self.map_name,
                                           tf_child_frame=self.odom_link_name)
        self.sync_joint_state_topic('/soar/joint_states')
        self.sync_odometry_topic('/soar/odom', self.drive_joint_name)
        fill_velocity_values = False
        self.add_follow_joint_trajectory_server(namespace='/soar/torso_trajectory_position_controller',
                                                fill_velocity_values=fill_velocity_values)
        self.add_follow_joint_trajectory_server(namespace='/soar/arm_trajectory_position_controller',
                                                fill_velocity_values=fill_velocity_values)
        self.add_follow_joint_trajectory_server(namespace='/soar/head_trajectory_position_controller',
                                                fill_velocity_values=fill_velocity_values)
        self.add_base_cmd_velocity(cmd_vel_topic='/soar/diff_drive_controller/cmd_vel',
                                   track_only_velocity=True,
                                   joint_name=self.drive_joint_name)


class SOARJointTrajServerUnrealInterface(RobotInterfaceConfig):
    map_name: str
    localization_joint_name: str
    odom_link_name: str
    drive_joint_name: str

    def __init__(self,
                 map_name: str = 'map',
                 localization_joint_name: str = 'localization',
                 odom_link_name: str = 'odom_combined',
                 drive_joint_name: str = 'brumbrum'):
        self.map_name = map_name
        self.localization_joint_name = localization_joint_name
        self.odom_link_name = odom_link_name
        self.drive_joint_name = drive_joint_name

    def setup(self):
        self.sync_6dof_joint_with_tf_frame(joint_name=self.localization_joint_name,
                                           tf_parent_frame=self.map_name,
                                           tf_child_frame=self.odom_link_name)
        self.sync_joint_state_topic('/soar/joint_states')
        self.sync_odometry_topic('/soar/odom', self.drive_joint_name)
        fill_velocity_values = False
        self.add_follow_joint_trajectory_server(namespace='/soar/whole_body_controller',
                                                fill_velocity_values=fill_velocity_values)
        self.add_follow_joint_trajectory_server(namespace='/soar/head_trajectory_controller',
                                                fill_velocity_values=fill_velocity_values)
        self.add_base_cmd_velocity(cmd_vel_topic='/soar/diff_drive_controller/cmd_vel',
                                   track_only_velocity=True,
                                   joint_name=self.drive_joint_name)
