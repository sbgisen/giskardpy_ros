#!/usr/bin/env python
import rospy

from giskardpy.qp.qp_controller_config import QPControllerConfig
from giskardpy_ros.configs.behavior_tree_config import OpenLoopBTConfig
from giskardpy_ros.configs.giskard import Giskard
from giskardpy_ros.configs.other_robots.soar import SOARCollisionAvoidance, WorldWithSOARConfig, SOARJointTrajServerIAIInterface
from giskardpy_ros.ros1.interface import ROS1Wrapper
from giskardpy.middleware import set_middleware
from giskardpy_ros.tree.behaviors.tf_publisher import TfPublishingModes
from giskardpy.model.world_config import WorldWithDiffDriveRobot



class WorldWithSOARConfigBlue(WorldWithSOARConfig):

    def setup(self):
        super().setup()
        # self.set_default_color(20 / 255, 27.1 / 255, 80 / 255, 0.2)


class OpenLoopBTConfigSOAR(OpenLoopBTConfig):
    def setup(self):
        # self.add_visualization_marker_publisher(add_to_sync=True, add_to_control_loop=True,
        #                                         mode=self.visualization_mode, include_tf_predix=True)
        self.add_gantt_chart_plotter()
        self.add_goal_graph_plotter()
        # self.add_tf_publisher(include_prefix=True, mode=TfPublishingModes.all)
        # self.add_trajectory_plotter(wait=True)
        # self.add_debug_trajectory_plotter(wait=True)
        self.add_debug_marker_publisher()
        # self.add_qp_data_publisher(
        #     publish_debug=True,
        #     publish_xdot=True,
        #     # publish_lbA=True,
        #     # publish_ubA=True
        # )


if __name__ == '__main__':
    rospy.init_node('giskard')
    set_middleware(ROS1Wrapper())
    giskard = Giskard(world_config=WorldWithDiffDriveRobot(urdf=rospy.get_param('robot_description')),
                      collision_avoidance_config=SOARCollisionAvoidance(),
                      robot_interface_config=SOARJointTrajServerIAIInterface(),
                      behavior_tree_config=OpenLoopBTConfig(debug_mode=True),
                      qp_controller_config=QPControllerConfig(mpc_dt=0.05))
    giskard.live()
