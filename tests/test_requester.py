#!/usr/bin/env python

# enable some python3 compatibility options:
from __future__ import absolute_import, print_function, unicode_literals

import uuid                     # standard Python module
import sys
import unittest
import rospy

from uuid_msgs.msg import UniqueID
import unique_id

# module being tested:
import scheduler_request_manager.scheduler as scheduler
import scheduler_request_manager.requester as requester

class TestSchedulerRequestManager(unittest.TestCase):
    """Unit tests for rocon scheduler request manager.
    """

    def test_empty_request(self):
        """ Issue empty allocation request.
        """
        r = requester.Requester()
        rospy.sleep(4.0)        # run requester for a while

def run_tests():
    # run the tests in this thread
    import rosunit
    try:
        rosunit.unitrun('scheduler_request_manager',
                        'test_scheduler_request_manager_py',
                        TestSchedulerRequestManager)
    finally:
        rospy.signal_shutdown('test complete') # terminate the test node

if __name__ == '__main__':

    rospy.init_node("test_requester")

    # create asynchronous thread for running the tests
    import threading
    test_th = threading.Thread(name='test_thread', target=run_tests)
    test_th.start()

    # spin in the main thread: required for message callbacks
    rospy.spin()
