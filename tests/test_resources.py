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

#RQR_UUID = unique_id.fromURL('package://rocon_scheduler_requests/requester')
TEST_UUID = unique_id.fromURL('package://rocon_scheduler_requests/test_uuid')
DIFF_UUID = unique_id.fromURL('package://rocon_scheduler_requests/diff_uuid')
TEST_RAPP = 'test/rapp'
TEST_RESOURCE = Resource(platform_info='linux.precise.ros.segbot.roberto',
                         name=TEST_RAPP, id=unique_id.toMsg(TEST_UUID))
TEST_RESOURCE_NAME = 'rocon:///linux.precise.ros.segbot.roberto'
TEST_RESOURCE_STRING = """rocon:///linux.precise.ros.segbot.roberto
  rapp: test/rapp
  id: """ + str(TEST_UUID) + """
  remappings:"""
TEST_WILDCARD = Resource(name=TEST_RAPP,
                         platform_info='linux.precise.ros.segbot.*')
TEST_WILDCARD_NAME = 'rocon:///linux.precise.ros.segbot.*'
TEST_WILDCARD_STRING = """rocon:///linux.precise.ros.segbot.*
  rapp: test/rapp
  id: 00000000-0000-0000-0000-000000000000
  remappings:"""


class TestResources(unittest.TestCase):
    """Unit tests for resource state transitions.

    These tests do not require a running ROS core.
    """

    ####################
    # resource tests
    ####################

    def test_rocon_name(self):
        self.assertEqual(rocon_name(TEST_RESOURCE), TEST_RESOURCE_NAME)
        self.assertEqual(rocon_name(TEST_WILDCARD), TEST_WILDCARD_NAME)

    def test_constructor(self):
        res1 = RoconResource(TEST_WILDCARD)
        self.assertIsNotNone(res1)
        self.assertEqual(res1.rocon_name(), TEST_WILDCARD_NAME)
        self.assertEqual(res1.msg, TEST_WILDCARD)
        self.assertEqual(str(res1), TEST_WILDCARD_STRING)

        res2 = RoconResource(TEST_RESOURCE)
        self.assertEqual(res2.msg, TEST_RESOURCE)
        self.assertEqual(res2.rocon_name(), TEST_RESOURCE_NAME)
        self.assertEqual(res2.msg, TEST_RESOURCE)
        self.assertEqual(str(res2), TEST_RESOURCE_STRING)

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
        self.assertTrue(TEST_RESOURCE_NAME not in res_set)
        self.assertIsNone(res_set.get(TEST_RESOURCE_NAME))
        self.assertEqual(res_set.get(TEST_RESOURCE_NAME, 3.14), 3.14)

        # Test equality for empty res_sets.
        self.assertTrue(res_set == ResourceSet([]))
        self.assertFalse(not res_set == ResourceSet([]))
        self.assertTrue((res_set != ResourceSet([TEST_RESOURCE])))
        self.assertFalse(res_set == ResourceSet([TEST_RESOURCE]))

    def test_one_resource_set(self):
        res_set = ResourceSet([TEST_RESOURCE])
        self.assertEqual(len(res_set), 1)
        self.assertTrue(RoconResource(TEST_RESOURCE) in res_set)
        self.assertTrue(rocon_name(TEST_RESOURCE) in res_set)
        self.assertEqual(res_set.get(TEST_RESOURCE_NAME),
                         res_set[TEST_RESOURCE_NAME])
        self.assertEqual(res_set.get(TEST_RESOURCE_NAME, 3.14),
                         res_set[TEST_RESOURCE_NAME])
        self.assertFalse('' in res_set)
        self.assertTrue(TEST_RESOURCE_NAME in res_set)
        self.assertFalse(TEST_WILDCARD_NAME in res_set)

        # Test equality for non-empty res_sets.
        self.assertFalse(res_set == ResourceSet([]))
        self.assertTrue(not res_set == ResourceSet([]))
        self.assertFalse((res_set != ResourceSet([TEST_RESOURCE])))
        self.assertTrue(res_set == ResourceSet([TEST_RESOURCE]))

    def test_two_resource_set(self):
        res_set = ResourceSet()
        self.assertEqual(len(res_set), 0)
        self.assertFalse(TEST_RESOURCE_NAME in res_set)
        self.assertFalse(TEST_WILDCARD_NAME in res_set)

        res_set[TEST_RESOURCE_NAME] = RoconResource(TEST_RESOURCE)
        self.assertEqual(len(res_set), 1)
        self.assertTrue(TEST_RESOURCE_NAME in res_set)
        self.assertFalse(TEST_WILDCARD_NAME in res_set)

        res_set[TEST_WILDCARD_NAME] = TEST_WILDCARD
        self.assertEqual(len(res_set), 2)
        self.assertTrue(TEST_RESOURCE_NAME in res_set)
        self.assertTrue(TEST_WILDCARD_NAME in res_set)

        # Test equality for res_set.
        self.assertFalse(res_set == ResourceSet([]))
        self.assertTrue(not res_set == ResourceSet([]))
        self.assertFalse(res_set != ResourceSet([TEST_RESOURCE, TEST_WILDCARD]))
        self.assertTrue(res_set == ResourceSet([TEST_RESOURCE, TEST_WILDCARD]))

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('rocon_scheduler_requests',
                    'test_transitions',
                    TestResources)
    rosunit.unitrun('rocon_scheduler_requests',
                    'test_request_sets',
                    TestResourceSets)
