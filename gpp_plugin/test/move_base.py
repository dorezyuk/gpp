#!/usr/bin/env python3

import rospy
import rostest
import unittest

from actionlib.simple_action_client import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction


class TestMoveBase(unittest.TestCase):
    """Verifies that move_base can handle our plugin"""

    def test_server_online(self):
        """Check that server is online

        move-base is very opaque compared to the move-base-flex framework;
        We cannot simply test, if a planner has been loaded successfully.

        However, we know that the move_base node crashes and dies, if the 
        planner is ill-defined.
        Hence, we can just test, if the main action is online, to verify, that
        the loading-process finished properly.
        """
        move_base = SimpleActionClient('/move_base', MoveBaseAction)
        self.assertTrue(move_base.wait_for_server(rospy.Duration(60)),
                        "{} server offline".format(move_base.action_client.ns))


if __name__ == '__main__':
    rospy.init_node("test_move_base")
    rostest.rosrun('gpp_plugin', 'test_move_base', TestMoveBase)
