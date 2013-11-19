#!/usr/bin/env python

# enable some python3 compatibility options:
# (unicode_literals not compatible with python2 uuid module)
from __future__ import absolute_import, print_function

import uuid
import unittest

# ROS dependencies
import unique_id
from rocon_std_msgs.msg import PlatformInfo
from scheduler_msgs.msg import Request

# module being tested:
from scheduler_request_manager.transitions import *

TEST_UUID = uuid.UUID('01234567-89ab-cdef-fedc-ba9876543210')
TEST_NEW_MSG = Request(id=unique_id.toMsg(TEST_UUID))
TEST_RESOURCE = PlatformInfo(os=PlatformInfo.OS_LINUX,
                             version=PlatformInfo.VERSION_UBUNTU_PRECISE,
                             system=PlatformInfo.SYSTEM_ROS,
                             platform='segbot',
                             name='roberto')

class TestTransitions(unittest.TestCase):
    """Unit tests for scheduler request state transitions.

    These tests do not require a running ROS core.
    """

    def test_constructor(self):
        rq = ResourceRequest()
        self.assertIsNotNone(rq)
        self.assertEqual(rq.get_status(), Request.NEW)

    def test_grant(self):
        rq = ResourceRequest()
        self.assertEqual(rq.get_status(), Request.NEW)
        rq.grant(TEST_RESOURCE)
        self.assertEqual(rq.get_status(), Request.GRANTED)

    def test_release(self):
        rq = ResourceRequest()
        self.assertEqual(rq.get_status(), Request.NEW)
        rq.grant(TEST_RESOURCE)
        self.assertEqual(rq.get_status(), Request.GRANTED)
        rq.release()
        self.assertEqual(rq.get_status(), Request.RELEASING)

    #def test_update(self):
    #    rq = transitions.Request(TEST_NEW_MSG)
    #    rq.update()

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('scheduler_request_transitions',
                    'test_transitions',
                    TestTransitions)
