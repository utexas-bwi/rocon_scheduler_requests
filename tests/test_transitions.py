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
DIFF_UUID = uuid.UUID('01234567-cdef-fedc-89ab-ba9876543210')
TEST_RESOURCE = PlatformInfo(os='linux',
                             version='precise',
                             system='ros',
                             platform='segbot',
                             name='roberto')
TEST_WILDCARD = PlatformInfo(os='linux',
                             version='precise',
                             system='ros',
                             platform='segbot',
                             name=PlatformInfo.NAME_ANY)
REQ_MSG1 = Request(id=unique_id.toMsg(TEST_UUID),
                   resource=TEST_WILDCARD,
                   status=Request.NEW)
REQ_MSG2 = Request(id=unique_id.toMsg(TEST_UUID),
                   resource=TEST_RESOURCE,
                   status=Request.NEW)


class TestTransitions(unittest.TestCase):
    """Unit tests for scheduler request state transitions.

    These tests do not require a running ROS core.
    """

    def test_to_Request(self):
        msg1 = to_Request(TEST_WILDCARD, uuid=TEST_UUID)
        self.assertEqual(msg1, 
                         Request(id=unique_id.toMsg(TEST_UUID),
                                 resource=TEST_WILDCARD,
                                 status=Request.NEW))
        msg2 = to_Request(TEST_RESOURCE, uuid=TEST_UUID)
        self.assertEqual(msg2, 
                         Request(id=unique_id.toMsg(TEST_UUID),
                                 resource=TEST_RESOURCE,
                                 status=Request.NEW))

    def test_constructor(self):
        rq1 = ResourceRequest(Request(id=unique_id.toMsg(TEST_UUID),
                                      resource=TEST_WILDCARD,
                                      status=Request.NEW))
        self.assertIsNotNone(rq1)
        self.assertEqual(rq1.get_status(), Request.NEW)
        self.assertEqual(rq1.get_resource(), TEST_WILDCARD)
        self.assertNotEqual(rq1.get_uuid, TEST_UUID)
        rq2 = ResourceRequest(Request(id=unique_id.toMsg(TEST_UUID),
                                      resource=TEST_RESOURCE,
                                      status=Request.NEW))
        self.assertEqual(rq2.get_status(), Request.NEW)
        self.assertEqual(rq2.get_resource(), TEST_RESOURCE)
        self.assertEqual(rq2.get_uuid(), TEST_UUID)

    def test_grant(self):
        rq = ResourceRequest(Request(id=unique_id.toMsg(TEST_UUID),
                                     resource=TEST_WILDCARD,
                                     status=Request.NEW))
        self.assertEqual(rq.get_status(), Request.NEW)
        rq.grant(TEST_RESOURCE)
        self.assertEqual(rq.get_status(), Request.GRANTED)
        self.assertEqual(rq.get_resource(), TEST_RESOURCE)

    def test_release(self):
        rq = ResourceRequest(Request(id=unique_id.toMsg(TEST_UUID),
                                     resource=TEST_WILDCARD,
                                     status=Request.NEW))
        self.assertEqual(rq.get_status(), Request.NEW)
        rq.grant(TEST_RESOURCE)
        self.assertEqual(rq.get_status(), Request.GRANTED)
        rq.release()
        self.assertEqual(rq.get_status(), Request.RELEASING)

    def test_free(self):
        rq = ResourceRequest(Request(id=unique_id.toMsg(TEST_UUID),
                                     resource=TEST_RESOURCE,
                                     status=Request.NEW))
        self.assertEqual(rq.get_status(), Request.NEW)
        rq.grant(TEST_RESOURCE)
        self.assertEqual(rq.get_status(), Request.GRANTED)
        rq.release()
        self.assertEqual(rq.get_status(), Request.RELEASING)
        rq.free()
        self.assertEqual(rq.get_status(), Request.RELEASED)

    def test_matches(self):
        rq = ResourceRequest(Request(id=unique_id.toMsg(TEST_UUID),
                                     resource=TEST_WILDCARD,
                                     status=Request.NEW))
        self.assertTrue(rq.matches(TEST_RESOURCE))
        kobuki = (PlatformInfo(os='linux',
                               version='precise',
                               system='ros',
                               platform='kobuki',
                               name='roberto'))
        self.assertFalse(rq.matches(kobuki))

    def test_empty_request_set(self):
        rset = RequestSet()
        self.assertIsNotNone(rset)
        self.assertEqual(len(rset), 0)
        self.assertTrue(TEST_UUID not in rset)

    def test_one_request_set(self):
        msg1 = Request(id=unique_id.toMsg(TEST_UUID),
                       resource=TEST_WILDCARD,
                       status=Request.NEW)
        rset = RequestSet([msg1])
        self.assertEqual(len(rset), 1)
        self.assertTrue(TEST_UUID in rset)
        self.assertEqual(rset[TEST_UUID].msg, msg1)
        self.assertEqual(rset.get(TEST_UUID), rset[TEST_UUID])
        self.assertFalse(DIFF_UUID in rset)
        self.assertIsNone(rset.get(DIFF_UUID))
        self.assertEqual(rset.get(DIFF_UUID, 10), 10)

    def test_empty_merge(self):
        msg1 = Request(id=unique_id.toMsg(TEST_UUID),
                       resource=TEST_WILDCARD,
                       status=Request.NEW)
        rset = RequestSet([msg1])
        self.assertEqual(len(rset), 1)
        self.assertTrue(TEST_UUID in rset)
        self.assertEqual([msg1], rset.list_requests())

        # merge an empty request set: rset should remain the same
        rset.merge(RequestSet())
        self.assertEqual(len(rset), 1)
        self.assertTrue(TEST_UUID in rset)
        self.assertEqual([msg1], rset.list_requests())

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('scheduler_request_transitions',
                    'test_transitions',
                    TestTransitions)
