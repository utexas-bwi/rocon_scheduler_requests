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
import rocon_scheduler_requests.scheduler as scheduler
import rocon_scheduler_requests.requester as requester

class TestSchedulerRequestManager(unittest.TestCase):
    """Unit tests for rocon scheduler request manager.
    """

    def test_empty_request(self):
        """ Issue empty allocation request at 1Hz frequency. """
        r = requester.Requester(frequency=1.0)
        rospy.sleep(4.0)        # run requester for a while

def run_tests():
    # run the tests in this thread
    import rosunit
    try:
        rosunit.unitrun('rocon_scheduler_requests',
                        'test_rocon_scheduler_requests_py',
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
