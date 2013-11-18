#!/usr/bin/env python

# enable some python3 compatibility options:
# (unicode_literals not compatible with python2 uuid module)
from __future__ import absolute_import, print_function

import uuid
import unittest

# ROS dependencies
import unique_id
from scheduler_msgs.msg import Request

# module being tested:
from scheduler_request_manager.transitions import *

TEST_UUID = uuid.UUID('01234567-89ab-cdef-fedc-ba9876543210')
TEST_NEW_MSG = Request(id=unique_id.toMsg(TEST_UUID))

class TestTransitions(unittest.TestCase):
    """Unit tests for scheduler request state transitions.

    These tests do not require a running ROS core.
    """

    def test_constructor(self):
        rq = ResourceRequest()
        self.assertIsNotNone(rq)
        self.assertEqual(rq.get_status(), Request.NEW)

    #def test_update(self):
    #    rq = transitions.Request(TEST_NEW_MSG)
    #    rq.update()

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('scheduler_request_transitions',
                    'test_transitions',
                    TestTransitions)
