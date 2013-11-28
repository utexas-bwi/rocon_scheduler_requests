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
from rocon_scheduler_requests.transitions import *

RQR_UUID = uuid.UUID('01234567-89ab-cdef-0123-456789abcdef')
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

    def test_abort(self):
        rq1 = ResourceRequest(Request(id=unique_id.toMsg(TEST_UUID),
                                      resource=TEST_RESOURCE,
                                      status=Request.NEW))
        rq1.abort()
        self.assertEqual(rq1.msg.status, Request.ABORTED)

        rq2 = ResourceRequest(Request(id=unique_id.toMsg(TEST_UUID),
                                      resource=TEST_RESOURCE,
                                      status=Request.WAITING))
        rq2.abort()
        self.assertEqual(rq2.msg.status, Request.ABORTED)

        rq3 = ResourceRequest(Request(id=unique_id.toMsg(TEST_UUID),
                                      resource=TEST_RESOURCE,
                                      status=Request.PREEMPTING))
        rq3.abort()
        self.assertEqual(rq3.msg.status, Request.ABORTED)

        rq4 = ResourceRequest(Request(id=unique_id.toMsg(TEST_UUID),
                                      resource=TEST_RESOURCE,
                                      status=Request.PREEMPTED))
        rq4.abort()
        self.assertEqual(rq4.msg.status, Request.ABORTED)

        rq5 = ResourceRequest(Request(id=unique_id.toMsg(TEST_UUID),
                                      resource=TEST_WILDCARD,
                                      status=Request.GRANTED))
        rq5.abort()
        self.assertEqual(rq5.msg.status, Request.ABORTED)

    def test_constructor(self):
        msg1 = Request(id=unique_id.toMsg(TEST_UUID),
                       resource=TEST_WILDCARD,
                       status=Request.NEW)
        rq1 = ResourceRequest(msg1)
        self.assertIsNotNone(rq1)
        self.assertNotEqual(str(rq1), str(msg1))
        self.assertEqual(str(rq1),
                         'id: 01234567-89ab-cdef-fedc-ba9876543210\n'
                         '    resource: linux.precise.ros.segbot.*\n'
                         '    status: 0')
        self.assertEqual(rq1.msg.status, Request.NEW)
        self.assertEqual(rq1.msg.resource, TEST_WILDCARD)
        self.assertEqual(rq1.str_resource(), 'linux.precise.ros.segbot.*')

        # why is this broken???
        #self.assertEqual(rq1.get_uuid, TEST_UUID)

        rq2 = ResourceRequest(Request(id=unique_id.toMsg(DIFF_UUID),
                                      resource=TEST_RESOURCE,
                                      status=Request.NEW))
        self.assertEqual(rq2.msg.status, Request.NEW)
        self.assertEqual(rq2.msg.resource, TEST_RESOURCE)
        self.assertEqual(rq2.get_uuid(), DIFF_UUID)
        self.assertEqual(rq2.str_resource(),
                         'linux.precise.ros.segbot.roberto')
        self.assertEqual(str(rq2),
                         'id: 01234567-cdef-fedc-89ab-ba9876543210\n'
                         '    resource: linux.precise.ros.segbot.roberto\n'
                         '    status: 0')

    def test_grant(self):
        rq1 = ResourceRequest(Request(id=unique_id.toMsg(TEST_UUID),
                                      resource=TEST_WILDCARD,
                                      status=Request.NEW))
        rq1.grant(TEST_RESOURCE)
        self.assertEqual(rq1.msg.status, Request.GRANTED)
        self.assertEqual(rq1.msg.resource, TEST_RESOURCE)

        rq2 = ResourceRequest(Request(id=unique_id.toMsg(TEST_UUID),
                                      resource=TEST_WILDCARD,
                                      status=Request.WAITING))
        rq2.grant(TEST_RESOURCE)
        self.assertEqual(rq2.msg.status, Request.GRANTED)
        self.assertEqual(rq2.msg.resource, TEST_RESOURCE)

        rq3 = ResourceRequest(Request(id=unique_id.toMsg(TEST_UUID),
                                      resource=TEST_WILDCARD,
                                      status=Request.PREEMPTING))
        self.assertRaises(TransitionError, rq3.grant, TEST_RESOURCE)

        rq4 = ResourceRequest(Request(id=unique_id.toMsg(TEST_UUID),
                                      resource=TEST_WILDCARD,
                                      status=Request.ABORTED))
        self.assertRaises(TransitionError, rq4.grant, TEST_RESOURCE)

    def test_reject(self):
        rq1 = ResourceRequest(Request(id=unique_id.toMsg(TEST_UUID),
                                      resource=TEST_RESOURCE,
                                      status=Request.NEW))
        rq1.reject()
        self.assertEqual(rq1.msg.status, Request.REJECTED)

        rq2 = ResourceRequest(Request(id=unique_id.toMsg(TEST_UUID),
                                      resource=TEST_RESOURCE,
                                      status=Request.WAITING))
        rq2.reject()
        self.assertEqual(rq2.msg.status, Request.REJECTED)

        rq3 = ResourceRequest(Request(id=unique_id.toMsg(TEST_UUID),
                                      resource=TEST_RESOURCE,
                                      status=Request.PREEMPTING))
        rq3.reject()
        self.assertEqual(rq3.msg.status, Request.REJECTED)

        rq4 = ResourceRequest(Request(id=unique_id.toMsg(TEST_UUID),
                                      resource=TEST_RESOURCE,
                                      status=Request.PREEMPTED))
        rq4.reject()
        self.assertEqual(rq4.msg.status, Request.REJECTED)

        rq5 = ResourceRequest(Request(id=unique_id.toMsg(TEST_UUID),
                                      resource=TEST_WILDCARD,
                                      status=Request.GRANTED))
        self.assertRaises(TransitionError, rq5.reject)

    def test_release(self):
        rq1 = ResourceRequest(Request(id=unique_id.toMsg(TEST_UUID),
                                      resource=TEST_RESOURCE,
                                      status=Request.GRANTED))
        rq1.release()
        self.assertEqual(rq1.msg.status, Request.RELEASING)

        rq2 = ResourceRequest(Request(id=unique_id.toMsg(TEST_UUID),
                                      resource=TEST_RESOURCE,
                                      status=Request.WAITING))
        rq2.release()
        self.assertEqual(rq2.msg.status, Request.RELEASING)

        rq3 = ResourceRequest(Request(id=unique_id.toMsg(TEST_UUID),
                                      resource=TEST_RESOURCE,
                                      status=Request.PREEMPTING))
        rq3.release()
        self.assertEqual(rq3.msg.status, Request.RELEASING)

        rq4 = ResourceRequest(Request(id=unique_id.toMsg(TEST_UUID),
                                      resource=TEST_RESOURCE,
                                      status=Request.ABORTED))
        self.assertRaises(TransitionError, rq4.release)

    def test_free(self):
        rq = ResourceRequest(Request(id=unique_id.toMsg(TEST_UUID),
                                     resource=TEST_RESOURCE,
                                     status=Request.NEW))
        rq.grant(TEST_RESOURCE)
        self.assertEqual(rq.msg.status, Request.GRANTED)
        rq.release()
        self.assertEqual(rq.msg.status, Request.RELEASING)
        rq.free()
        self.assertEqual(rq.msg.status, Request.RELEASED)

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
        rset = RequestSet([], RQR_UUID)
        self.assertIsNotNone(rset)
        self.assertEqual(len(rset), 0)
        self.assertTrue(TEST_UUID not in rset)
        self.assertEqual([], rset.list_requests())
        rset_str = """requester_id: 01234567-89ab-cdef-0123-456789abcdef
priority: 0
requests:"""
        self.assertEqual(str(rset), rset_str)

    def test_one_request_set(self):
        msg1 = Request(id=unique_id.toMsg(TEST_UUID),
                       resource=TEST_WILDCARD,
                       status=Request.NEW)
        rset = RequestSet([msg1], RQR_UUID)
        self.assertEqual(len(rset), 1)
        self.assertTrue(TEST_UUID in rset)
        self.assertEqual(rset[TEST_UUID].msg, msg1)
        self.assertEqual(rset.get(TEST_UUID), rset[TEST_UUID])
        self.assertFalse(DIFF_UUID in rset)
        self.assertIsNone(rset.get(DIFF_UUID))
        self.assertEqual(rset.get(DIFF_UUID, 10), 10)
        self.assertEqual([msg1], rset.list_requests())
        rset_str = """requester_id: 01234567-89ab-cdef-0123-456789abcdef
priority: 0
requests:
  id: 01234567-89ab-cdef-fedc-ba9876543210
    resource: linux.precise.ros.segbot.*
    status: 0"""
        self.assertEqual(str(rset), rset_str)

    def test_two_request_set(self):
        msg1 = Request(id=unique_id.toMsg(TEST_UUID),
                       resource=TEST_WILDCARD)
        msg2 = Request(id=unique_id.toMsg(DIFF_UUID),
                       resource=TEST_RESOURCE)
        rset = RequestSet([msg1, msg2], RQR_UUID)
        self.assertEqual(len(rset), 2)
        self.assertTrue(TEST_UUID in rset)
        self.assertTrue(DIFF_UUID in rset)
        self.assertEqual(rset[TEST_UUID].msg, msg1)
        self.assertEqual(rset[DIFF_UUID].msg, msg2)
        self.assertEqual(rset.get(TEST_UUID), rset[TEST_UUID])
        self.assertEqual(rset.get(DIFF_UUID), rset[DIFF_UUID])

    def test_empty_merge(self):
        msg1 = Request(id=unique_id.toMsg(TEST_UUID),
                       resource=TEST_WILDCARD,
                       status=Request.NEW)
        rset = RequestSet([msg1], RQR_UUID)
        self.assertEqual(len(rset), 1)
        self.assertTrue(TEST_UUID in rset)
        self.assertEqual([msg1], rset.list_requests())

        # merge an empty request set: rset should remain the same
        rset.merge(RequestSet([], RQR_UUID))
        self.assertEqual(len(rset), 1)
        self.assertTrue(TEST_UUID in rset)
        self.assertEqual([msg1], rset.list_requests())

    def test_single_merge(self):
        msg1 = Request(id=unique_id.toMsg(TEST_UUID),
                       resource=TEST_WILDCARD,
                       status=Request.NEW)
        rset = RequestSet([msg1], RQR_UUID)
        self.assertEqual(rset[TEST_UUID].msg.status, Request.NEW)
        self.assertEqual(rset[TEST_UUID].msg.resource, TEST_WILDCARD)
        self.assertEqual(rset[TEST_UUID].msg.id, unique_id.toMsg(TEST_UUID))

        # merge an empty request set: rset should remain the same
        msg2 = Request(id=unique_id.toMsg(TEST_UUID),
                       resource=TEST_RESOURCE,
                       status=Request.GRANTED)
        rset.merge(RequestSet([msg2], RQR_UUID))
        self.assertEqual(len(rset), 1)
        self.assertTrue(TEST_UUID in rset)
        self.assertEqual([msg1], rset.list_requests())
        self.assertEqual(rset[TEST_UUID].msg.status, Request.GRANTED)
        self.assertEqual(rset[TEST_UUID].msg.resource, TEST_RESOURCE)

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('scheduler_request_transitions',
                    'test_transitions',
                    TestTransitions)
