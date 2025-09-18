#!/usr/bin/env python
from giskardpy.model.collision_avoidance_config import CollisionAvoidanceConfig
from giskardpy.model.world_config import WorldWithDiffDriveRobot
from giskardpy.qp.qp_controller_config import QPControllerConfig
from rclpy import Parameter

from giskardpy_ros.configs.behavior_tree_config import OpenLoopBTConfig
from giskardpy_ros.configs.behavior_tree_config import StandAloneBTConfig
from giskardpy_ros.configs.giskard import Giskard
from giskardpy_ros.configs.robot_interface_config import RobotInterfaceConfig
from giskardpy_ros.configs.robot_interface_config import StandAloneRobotInterfaceConfig
from giskardpy_ros.ros2 import rospy


class WorldWithSOARConfig(WorldWithDiffDriveRobot):

    def __init__(self,
                 urdf,
                 map_name='map',
                 localization_joint_name='localization',
                 odom_link_name='odom',
                 drive_joint_name='brumbrum'):
        super().__init__(urdf=urdf,
                         map_name=map_name,
                         localization_joint_name=localization_joint_name,
                         odom_link_name=odom_link_name,
                         drive_joint_name=drive_joint_name)


class SOARStandaloneInterface(StandAloneRobotInterfaceConfig):

    def __init__(self, drive_joint_name='brumbrum'):
        super().__init__([
            drive_joint_name, 'wheel_left_joint', 'wheel_right_joint', 'torso_joint', 'head_pan_joint',
            'head_tilt_joint', 'arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4', 'arm_joint5', 'arm_joint6',
            'arm_joint7', 'arm_drive_joint'
        ])


class SOARJointTrajServer(RobotInterfaceConfig):

    def __init__(self,
                 map_name='map',
                 localization_joint_name='localization',
                 odom_link_name='odom',
                 drive_joint_name='brumbrum'):
        super().__init__()
        self.map_name = map_name
        self.localization_joint_name = localization_joint_name
        self.odom_link_name = odom_link_name
        self.drive_joint_name = drive_joint_name

    def setup(self):
        self.sync_6dof_joint_with_tf_frame(joint_name=self.localization_joint_name,
                                           tf_parent_frame=self.map_name,
                                           tf_child_frame=self.odom_link_name)
        self.sync_joint_state_topic('/soar/joint_states')
        self.sync_odometry_topic('/soar/odom')
        self.add_follow_joint_trajectory_server(namespace='/soar/controller/torso_trajectory')
        self.add_follow_joint_trajectory_server(namespace='/soar/controller/arm_trajectory')
        self.add_base_cmd_velocity(cmd_vel_topic='/soar/controller/mobile_base/cmd_vel',
                                   track_only_velocity=True,
                                   joint_name=self.drive_joint_name)


class SOARCollisionAvoidanceConfig(CollisionAvoidanceConfig):

    def __init__(self, drive_joint_name='brumbrum'):
        super().__init__()
        self.drive_joint_name = drive_joint_name

    def setup(self):
        self.load_self_collision_matrix('package://soar_moveit_config/config/soar.srdf')
        self.overwrite_external_collision_avoidance(self.drive_joint_name,
                                                    number_of_repeller=2,
                                                    soft_threshold=0.2,
                                                    hard_threshold=0.1)


def main():
    rospy.init_node('giskard')
    rospy.node.declare_parameters(namespace='', parameters=[('robot_description', Parameter.Type.STRING)])
    robot_description = rospy.node.get_parameter_or('robot_description').value
    # giskard = Giskard(world_config=WorldWithSOARConfig(urdf=robot_description), collision_avoidance_config=SOARCollisionAvoidanceConfig(), robot_interface_config=SOARStandaloneInterface(), behavior_tree_config=StandAloneBTConfig(publish_tf=True, publish_js=False, debug_mode=True), qp_controller_config=QPControllerConfig())
    giskard = Giskard(world_config=WorldWithSOARConfig(urdf=robot_description),
                      collision_avoidance_config=SOARCollisionAvoidanceConfig(),
                      robot_interface_config=SOARJointTrajServer(),
                      behavior_tree_config=OpenLoopBTConfig(debug_mode=True),
                      qp_controller_config=QPControllerConfig())
    giskard.live()


if __name__ == '__main__':
    main()
