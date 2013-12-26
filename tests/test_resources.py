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
DIFF_UUID = unique_id.fromURL('package://rocon_scheduler_requests/diff_uuid')
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

    def test_allocate(self):
        res1 = RoconResource(TEST_RESOURCE)
        self.assertEqual(res1.status, AVAILABLE)
        self.assertEqual(res1.owner, None)
        res1.allocate(TEST_UUID)
        self.assertEqual(res1.status, ALLOCATED)
        self.assertEqual(res1.owner, TEST_UUID)
        self.assertRaises(ResourceNotAvailableError, res1.allocate, DIFF_UUID)
        self.assertRaises(ResourceNotAvailableError, res1.allocate, TEST_UUID)

    def test_release(self):
        res1 = RoconResource(TEST_RESOURCE)
        self.assertEqual(res1.status, AVAILABLE)
        self.assertEqual(res1.owner, None)
        res1.allocate(TEST_UUID)
        self.assertEqual(res1.status, ALLOCATED)
        self.assertEqual(res1.owner, TEST_UUID)
        self.assertRaises(ResourceNotOwnedError, res1.release, DIFF_UUID)
        res1.release(TEST_UUID)
        self.assertEqual(res1.status, AVAILABLE)

        res2 = RoconResource(TEST_RESOURCE)
        res2.allocate(TEST_UUID)
        self.assertEqual(res2.status, ALLOCATED)
        res2.status = MISSING           # resource now missing
        res2.release(TEST_UUID)
        self.assertEqual(res2.status, MISSING)


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
        self.assertFalse(RoconResource(TEST_RESOURCE) in res_set)
        self.assertTrue('arbitrary name' not in res_set)
        self.assertTrue('linux.precise.ros.segbot.roberto/test_rapp'
                        not in res_set)

        # Test equality for empty res_sets.
        self.assertTrue(res_set == ResourceSet([]))
        self.assertFalse(not res_set == ResourceSet([]))
        self.assertTrue((res_set != ResourceSet([TEST_RESOURCE])))
        self.assertFalse(res_set == ResourceSet([TEST_RESOURCE]))

    def test_one_resource_set(self):
        res_set = ResourceSet([TEST_RESOURCE])
        self.assertEqual(len(res_set), 1)
        self.assertTrue(RoconResource(TEST_RESOURCE) in res_set)
        self.assertTrue(str(RoconResource(TEST_RESOURCE)) in res_set)
        self.assertTrue('linux.precise.ros.segbot.roberto/test_rapp' in res_set)
        self.assertFalse('linux.precise.ros.segbot.*/test_rapp' in res_set)

        # Test equality for non-empty res_sets.
        self.assertFalse(res_set == ResourceSet([]))
        self.assertTrue(not res_set == ResourceSet([]))
        self.assertFalse((res_set != ResourceSet([TEST_RESOURCE])))
        self.assertTrue(res_set == ResourceSet([TEST_RESOURCE]))

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('rocon_scheduler_requests',
                    'test_transitions',
                    TestResources)
    rosunit.unitrun('rocon_scheduler_requests',
                    'test_request_sets',
                    TestResourceSets)
