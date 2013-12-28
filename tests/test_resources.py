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
TEST_RAPP = 'rocon_apps/teleop'
TEST_RESOURCE = Resource(
    platform_info='rocon:///linux.precise.ros.segbot.roberto',
    name=TEST_RAPP)
TEST_RESOURCE_NAME = 'rocon:///linux.precise.ros.segbot.roberto'
TEST_RESOURCE_STRING = """rocon:///linux.precise.ros.segbot.roberto, status: 0
  owner: None
  rapps:
    rocon_apps/teleop"""

# this Resource has an old-format platform_info string:
TEST_ANOTHER = Resource(
    platform_info='linux.precise.ros.segbot.marvin',
    name=TEST_RAPP)
TEST_ANOTHER_NAME = 'rocon:///linux.precise.ros.segbot.marvin'
TEST_ANOTHER_STRING = """rocon:///linux.precise.ros.segbot.marvin, status: 0
  owner: None
  rapps:
    rocon_apps/teleop"""

TEST_REGEX = Resource(name=TEST_RAPP,
                      platform_info=r'rocon:///linux\.precise\.ros\.segbot\..*')


class TestRoconResource(unittest.TestCase):
    """Unit tests for ROCON resource class.

    These tests do not require a running ROS core.
    """

    ####################
    # ROCON resource tests
    ####################

    def test_rocon_name(self):
        self.assertEqual(rocon_name(TEST_RESOURCE), TEST_RESOURCE_NAME)
        self.assertEqual(rocon_name(TEST_ANOTHER), TEST_ANOTHER_NAME)

    def test_constructor(self):
        res1 = RoconResource(TEST_ANOTHER)
        self.assertIsNotNone(res1)
        self.assertEqual(rocon_name(res1), TEST_ANOTHER_NAME)
        self.assertMultiLineEqual(str(res1), TEST_ANOTHER_STRING)

        res2 = RoconResource(TEST_RESOURCE)
        self.assertEqual(rocon_name(res2), TEST_RESOURCE_NAME)
        self.assertMultiLineEqual(str(res2), TEST_RESOURCE_STRING)

    def test_allocate(self):
        res1 = RoconResource(TEST_RESOURCE)
        self.assertEqual(res1.status, AVAILABLE)
        self.assertEqual(res1.owner, None)
        res1.allocate(TEST_UUID)
        self.assertEqual(res1.status, ALLOCATED)
        self.assertEqual(res1.owner, TEST_UUID)
        self.assertRaises(ResourceNotAvailableError, res1.allocate, DIFF_UUID)
        self.assertRaises(ResourceNotAvailableError, res1.allocate, TEST_UUID)

    def test_equality(self):
        res1 = RoconResource(TEST_RESOURCE)
        self.assertEqual(res1, RoconResource(TEST_RESOURCE))

        # different platform_info
        self.assertNotEqual(res1, RoconResource(TEST_ANOTHER))

        # different rapp name
        self.assertNotEqual(res1, RoconResource(Resource(
            platform_info='linux.precise.ros.segbot.roberto',
            name='other_package/teleop')))

        # different owner
        res2 = RoconResource(TEST_RESOURCE)
        res1.allocate(TEST_UUID)
        self.assertNotEqual(res1, res2)

        # different status
        res3 = RoconResource(TEST_RESOURCE)
        res3.status = MISSING
        self.assertNotEqual(res1, res3)

    def test_matches(self):
        res1 = RoconResource(TEST_RESOURCE)
        self.assertTrue(res1.match(TEST_REGEX))
        self.assertTrue(res1.match(TEST_RESOURCE))
        self.assertFalse(res1.match(Resource(
            name='different/rapp',
            platform_info=r'rocon:///linux\.precise\.ros\.segbot\..*')))
        self.assertFalse(res1.match(Resource(
            name=TEST_RAPP,
            platform_info=r'rocon:///linux\.precise\.ros\.segbot\.marvin')))
        self.assertTrue(res1.match(Resource(
            name=TEST_RAPP,
            platform_info=r'rocon:///linux\..*\.ros\.(segbot|turtlebot)\..*')))

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


class TestResourceSet(unittest.TestCase):
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
        self.assertEqual(str(res_set), 'ROCON resource set:')

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
        self.assertTrue(TEST_ANOTHER_NAME not in res_set)

        # Test equality for non-empty res_sets.
        self.assertFalse(res_set == ResourceSet([]))
        self.assertTrue(not res_set == ResourceSet([]))
        self.assertFalse((res_set != ResourceSet([TEST_RESOURCE])))
        self.assertTrue(res_set == ResourceSet([TEST_RESOURCE]))
        self.assertMultiLineEqual(
            str(res_set), 'ROCON resource set:\n  ' + TEST_RESOURCE_STRING)

    def test_two_resource_set(self):
        res_set = ResourceSet()
        self.assertEqual(len(res_set), 0)
        self.assertFalse(TEST_RESOURCE_NAME in res_set)
        self.assertFalse(TEST_ANOTHER_NAME in res_set)

        res_set[TEST_RESOURCE_NAME] = RoconResource(TEST_RESOURCE)
        self.assertEqual(len(res_set), 1)
        self.assertTrue(TEST_RESOURCE_NAME in res_set)
        self.assertFalse(TEST_ANOTHER_NAME in res_set)

        res_set[TEST_ANOTHER_NAME] = TEST_ANOTHER
        self.assertEqual(len(res_set), 2)
        self.assertTrue(TEST_RESOURCE_NAME in res_set)
        self.assertTrue(TEST_ANOTHER_NAME in res_set)

        # Test equality for res_set.
        self.assertFalse(res_set == ResourceSet([]))
        self.assertTrue(not res_set == ResourceSet([]))
        self.assertFalse(res_set != ResourceSet([TEST_RESOURCE, TEST_ANOTHER]))
        self.assertTrue(res_set == ResourceSet([TEST_RESOURCE, TEST_ANOTHER]))

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('rocon_scheduler_requests',
                    'test_transitions',
                    TestRoconResource)
    rosunit.unitrun('rocon_scheduler_requests',
                    'test_request_sets',
                    TestResourceSet)
