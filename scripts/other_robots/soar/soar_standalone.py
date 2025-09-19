#!/usr/bin/env python
import rospy

from giskardpy_ros.configs.behavior_tree_config import StandAloneBTConfig
from giskardpy_ros.configs.giskard import Giskard
from giskardpy_ros.configs.other_robots.soar import SOARCollisionAvoidance, WorldWithSOARConfig, SOARStandaloneInterface
from giskardpy.qp.qp_controller_config import QPControllerConfig, SupportedQPSolver
from giskardpy_ros.configs.robot_interface_config import StandAloneRobotInterfaceConfig
from giskardpy_ros.ros1.interface import ROS1Wrapper
from giskardpy.middleware import set_middleware

if __name__ == '__main__':
    rospy.init_node('giskard')
    set_middleware(ROS1Wrapper())
    drive_joint_name = 'brumbrum'
    giskard = Giskard(world_config=WorldWithSOARConfig(drive_joint_name=drive_joint_name),
                      collision_avoidance_config=SOARCollisionAvoidance(drive_joint_name=drive_joint_name),
                      robot_interface_config=SOARStandaloneInterface(drive_joint_name=drive_joint_name),
                      behavior_tree_config=StandAloneBTConfig(publish_tf=True, publish_js=True, debug_mode=True),
                      qp_controller_config=QPControllerConfig())
    giskard.live()
