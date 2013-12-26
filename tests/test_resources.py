#!/usr/bin/env python

# enable some python3 compatibility options:
# (unicode_literals not compatible with python2 uuid module)
from __future__ import absolute_import, print_function

import copy
import uuid
import unittest

# ROS dependencies
import unique_id
from scheduler_msgs.msg import Request
from scheduler_msgs.msg import Resource

# module being tested:
from rocon_scheduler_requests.resources import *

RQR_UUID = unique_id.fromURL('package://rocon_scheduler_requests/requester')
TEST_UUID = unique_id.fromURL('package://rocon_scheduler_requests/test_uuid')
TEST_UUID = unique_id.fromURL('package://rocon_scheduler_requests/diff_uuid')
TEST_RAPP = 'test_rapp'
TEST_RESOURCE = Resource(name=TEST_RAPP,
                         platform_info='linux.precise.ros.segbot.roberto')
TEST_WILDCARD = Resource(name=TEST_RAPP,
                         platform_info='linux.precise.ros.segbot.*')


class TestResources(unittest.TestCase):
    """Unit tests for resource state transitions.

    These tests do not require a running ROS core.
    """

    ####################
    # resource tests
    ####################

    def test_constructor(self):
        res1 = RoconResource(TEST_WILDCARD)
        self.assertIsNotNone(res1)
        self.assertEqual(str(res1), 'linux.precise.ros.segbot.*/test_rapp')
        self.assertEqual(res1.msg, TEST_WILDCARD)

        res2 = RoconResource(TEST_RESOURCE)
        self.assertEqual(res2.msg, TEST_RESOURCE)
        self.assertEqual(str(res2),
                         'linux.precise.ros.segbot.roberto/test_rapp')
        self.assertEqual(res2.msg, TEST_RESOURCE)


class TestResourceSets(unittest.TestCase):
    """Unit tests for resource set operations.

    These tests do not require a running ROS core.
    """

    ####################
    # resource set tests
    ####################

    def test_empty_resource_set(self):
        res_set = ResourceSet()
        self.assertIsNotNone(res_set)
        self.assertEqual(len(res_set), 0)
        #self.assertTrue('arbitrary name' not in res_set)

        # Test equality for empty res_sets.
        self.assertTrue(res_set == ResourceSet([]))
        self.assertFalse(not res_set == ResourceSet([]))
        self.assertTrue((res_set != ResourceSet([TEST_RESOURCE])))
        self.assertFalse(res_set == ResourceSet([TEST_RESOURCE]))

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('rocon_scheduler_requests',
                    'test_transitions',
                    TestResources)
    rosunit.unitrun('rocon_scheduler_requests',
                    'test_request_sets',
                    TestResourceSets)
