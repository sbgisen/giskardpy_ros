#!/usr/bin/env python
import rospy

from giskardpy.qp.qp_controller_config import QPControllerConfig
from giskardpy_ros.configs.behavior_tree_config import ClosedLoopBTConfig
from giskardpy_ros.configs.giskard import Giskard
from giskardpy_ros.configs.other_robots.soar import SOARCollisionAvoidance, WorldWithSOARConfig, SOARVelocityIAIInterface
from giskardpy_ros.ros1.interface import ROS1Wrapper
from giskardpy.middleware import set_middleware
from giskardpy_ros.tree.behaviors.tf_publisher import TfPublishingModes
from giskardpy.model.world_config import WorldWithDiffDriveRobot



if __name__ == '__main__':
    rospy.init_node('giskard')
    set_middleware(ROS1Wrapper())
    giskard = Giskard(world_config=WorldWithDiffDriveRobot(urdf=rospy.get_param('robot_description')),
                      collision_avoidance_config=SOARCollisionAvoidance(),
                      robot_interface_config=SOARVelocityIAIInterface(),
                      behavior_tree_config=ClosedLoopBTConfig(debug_mode=True),
                      qp_controller_config=QPControllerConfig(mpc_dt=0.05))
    giskard.live()
