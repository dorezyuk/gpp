#!/usr/bin/env python3

import rospy
import rostest
import unittest

from actionlib.simple_action_client import SimpleActionClient
from mbf_msgs.msg import GetPathAction, GetPathGoal, GetPathResult


class TestMbfCostmapNav(unittest.TestCase):
    """Verifies that move_base can handle our plugin"""

    def test_server_online(self):
        """Check that server is online
        
        In this simple setup, we don't really want to generate a plan.
        Instread, we can ask for a plan on our planner (gpp_gp), and verify,
        that the generated error-message is not INVALID_PLUGIN

        See https://github.com/magazino/move_base_flex/blob/596ed881bfcbd847e9d296c6d38e4d3fa3b74a4d/mbf_msgs/action/GetPath.action
        for reference.
        """
        # setup the client
        get_path = SimpleActionClient('move_base_flex/get_path', GetPathAction)
        self.assertTrue(get_path.wait_for_server(rospy.Duration(60)),
                        "{} server offline".format(get_path.action_client.ns))

        # send a dummy goal with the right planner
        goal = GetPathGoal()
        goal.planner = 'gpp_gp'
        get_path.send_goal_and_wait(goal, rospy.Duration(1))
        result = get_path.get_result()

        # we are happy as long the plugin is known
        self.assertNotEqual(result.outcome, GetPathResult.INVALID_PLUGIN)


if __name__ == '__main__':
    rospy.init_node("test_mbf_costmap_nav")
    rostest.rosrun('gpp_plugin', 'test_mbf_costmap_nav', TestMbfCostmapNav)
