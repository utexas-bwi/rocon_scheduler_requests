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
from rocon_scheduler_requests.transitions import *

RQR_UUID = uuid.UUID('01234567-89ab-cdef-0123-456789abcdef')
TEST_UUID = uuid.UUID('01234567-89ab-cdef-fedc-ba9876543210')
DIFF_UUID = uuid.UUID('01234567-cdef-fedc-89ab-ba9876543210')
TEST_RAPP = 'test_rapp'
TEST_RESOURCE = Resource(name=TEST_RAPP,
                         platform_info='linux.precise.ros.segbot.roberto')
TEST_WILDCARD = Resource(name=TEST_RAPP,
                         platform_info='linux.precise.ros.segbot.*')


class TestTransitions(unittest.TestCase):
    """Unit tests for scheduler request state transitions.

    These tests do not require a running ROS core.
    """

    def test_abort(self):
        rq1 = ResourceReply(Request(id=unique_id.toMsg(TEST_UUID),
                                    resources=[TEST_RESOURCE],
                                    status=Request.NEW))
        rq1.abort()
        self.assertEqual(rq1.msg.status, Request.ABORTED)

        rq2 = ResourceReply(Request(id=unique_id.toMsg(TEST_UUID),
                                    resources=[TEST_RESOURCE],
                                    status=Request.WAITING))
        rq2.abort()
        self.assertEqual(rq2.msg.status, Request.ABORTED)

        rq3 = ResourceReply(Request(id=unique_id.toMsg(TEST_UUID),
                                    resources=[TEST_RESOURCE],
                                    status=Request.PREEMPTING))
        rq3.abort()
        self.assertEqual(rq3.msg.status, Request.ABORTED)

        rq4 = ResourceReply(Request(id=unique_id.toMsg(TEST_UUID),
                                    resources=[TEST_RESOURCE],
                                    status=Request.PREEMPTED))
        rq4.abort()
        self.assertEqual(rq4.msg.status, Request.ABORTED)

        rq5 = ResourceReply(Request(id=unique_id.toMsg(TEST_UUID),
                                    resources=[TEST_WILDCARD],
                                    status=Request.GRANTED))
        rq5.abort()
        self.assertEqual(rq5.msg.status, Request.ABORTED)

    def test_constructor(self):
        msg1 = Request(id=unique_id.toMsg(TEST_UUID),
                       resources=[TEST_WILDCARD],
                       status=Request.NEW)
        rq1 = ResourceRequest(msg1)
        self.assertIsNotNone(rq1)
        self.assertEqual(str(rq1),
                         'id: 01234567-89ab-cdef-fedc-ba9876543210\n'
                         '    priority: 0\n'
                         '    resources: \n'
                         '      linux.precise.ros.segbot.*/test_rapp\n'
                         '    status: 0')
        self.assertEqual(rq1.msg.status, Request.NEW)
        self.assertEqual(rq1.msg.resources, [TEST_WILDCARD])

        # why is this broken???
        #self.assertEqual(rq1.get_uuid, TEST_UUID)

        rq2 = ResourceRequest(Request(id=unique_id.toMsg(DIFF_UUID),
                                      resources=[TEST_RESOURCE],
                                      status=Request.NEW))
        self.assertEqual(rq2.msg.status, Request.NEW)
        self.assertEqual(rq2.msg.resources, [TEST_RESOURCE])
        self.assertEqual(rq2.get_uuid(), DIFF_UUID)
        self.assertEqual(str(rq2),
                         'id: 01234567-cdef-fedc-89ab-ba9876543210\n'
                         '    priority: 0\n'
                         '    resources: \n'
                         '      linux.precise.ros.segbot.roberto/test_rapp\n'
                         '    status: 0')
    def test_grant(self):
        rq1 = ResourceReply(Request(id=unique_id.toMsg(TEST_UUID),
                                    resources=[TEST_WILDCARD],
                                    status=Request.NEW))
        rq1.grant([TEST_RESOURCE])
        self.assertEqual(rq1.msg.status, Request.GRANTED)
        self.assertEqual(rq1.msg.resources, [TEST_RESOURCE])

        rq2 = ResourceReply(Request(id=unique_id.toMsg(TEST_UUID),
                                    resources=[TEST_WILDCARD],
                                    status=Request.WAITING))
        rq2.grant([TEST_RESOURCE])
        self.assertEqual(rq2.msg.status, Request.GRANTED)
        self.assertEqual(rq2.msg.resources, [TEST_RESOURCE])

        rq3 = ResourceReply(Request(id=unique_id.toMsg(TEST_UUID),
                                    resources=[TEST_WILDCARD],
                                    status=Request.PREEMPTING))
        self.assertRaises(TransitionError, rq3.grant, [TEST_RESOURCE])

        rq4 = ResourceReply(Request(id=unique_id.toMsg(TEST_UUID),
                                    resources=[TEST_WILDCARD],
                                    status=Request.ABORTED))
        self.assertRaises(TransitionError, rq4.grant, [TEST_RESOURCE])

    def test_reject(self):
        rq1 = ResourceReply(Request(id=unique_id.toMsg(TEST_UUID),
                                    resources=[TEST_RESOURCE],
                                    status=Request.NEW))
        rq1.reject()
        self.assertEqual(rq1.msg.status, Request.REJECTED)

        rq2 = ResourceReply(Request(id=unique_id.toMsg(TEST_UUID),
                                    resources=[TEST_RESOURCE],
                                    status=Request.WAITING))
        rq2.reject()
        self.assertEqual(rq2.msg.status, Request.REJECTED)

        rq3 = ResourceReply(Request(id=unique_id.toMsg(TEST_UUID),
                                    resources=[TEST_RESOURCE],
                                    status=Request.PREEMPTING))
        rq3.reject()
        self.assertEqual(rq3.msg.status, Request.REJECTED)

        rq4 = ResourceReply(Request(id=unique_id.toMsg(TEST_UUID),
                                    resources=[TEST_RESOURCE],
                                    status=Request.PREEMPTED))
        rq4.reject()
        self.assertEqual(rq4.msg.status, Request.REJECTED)

        rq5 = ResourceReply(Request(id=unique_id.toMsg(TEST_UUID),
                                    resources=[TEST_WILDCARD],
                                    status=Request.GRANTED))
        self.assertRaises(TransitionError, rq5.reject)

    def test_release(self):
        rq1 = ResourceRequest(Request(id=unique_id.toMsg(TEST_UUID),
                                      resources=[TEST_RESOURCE],
                                      status=Request.GRANTED))
        rq1.release()
        self.assertEqual(rq1.msg.status, Request.RELEASING)

        rq2 = ResourceRequest(Request(id=unique_id.toMsg(TEST_UUID),
                                      resources=[TEST_RESOURCE],
                                      status=Request.WAITING))
        rq2.release()
        self.assertEqual(rq2.msg.status, Request.RELEASING)

        rq3 = ResourceRequest(Request(id=unique_id.toMsg(TEST_UUID),
                                      resources=[TEST_RESOURCE],
                                      status=Request.PREEMPTING))
        rq3.release()
        self.assertEqual(rq3.msg.status, Request.RELEASING)

        rq4 = ResourceRequest(Request(id=unique_id.toMsg(TEST_UUID),
                                      resources=[TEST_RESOURCE],
                                      status=Request.ABORTED))
        self.assertRaises(TransitionError, rq4.release)

    def test_free(self):
        rq = ResourceReply(Request(id=unique_id.toMsg(TEST_UUID),
                                   resources=[TEST_RESOURCE],
                                   status=Request.RELEASING))
        rq.free()
        self.assertEqual(rq.msg.status, Request.RELEASED)

    def test_empty_request_set(self):
        rset = RequestSet([], RQR_UUID)
        self.assertIsNotNone(rset)
        self.assertEqual(len(rset), 0)
        self.assertTrue(TEST_UUID not in rset)
        sch_msg = SchedulerRequests(requester=unique_id.toMsg(RQR_UUID),
                                    requests=[])
        self.assertEqual(rset.to_msg(stamp=rospy.Time()), sch_msg)
        rset_str = """requester_id: 01234567-89ab-cdef-0123-456789abcdef
requests:"""
        self.assertEqual(str(rset), rset_str)

        # Test equality for empty rsets: the requester_id is
        # significant, but the contents type is ignored.
        self.assertTrue(rset == RequestSet([], RQR_UUID))
        self.assertTrue(rset == RequestSet([], RQR_UUID,
                                           contents=ResourceReply))
        self.assertFalse((rset != RequestSet([], RQR_UUID)))
        self.assertFalse(not rset == RequestSet([], RQR_UUID))
        self.assertFalse(rset == RequestSet([], TEST_UUID))
        self.assertTrue(rset != RequestSet([], TEST_UUID))

    def test_one_request_set(self):
        msg1 = Request(id=unique_id.toMsg(TEST_UUID),
                       resources=[TEST_WILDCARD],
                       status=Request.NEW)
        rset = RequestSet([msg1], RQR_UUID, contents=ResourceReply)
        self.assertEqual(len(rset), 1)
        self.assertTrue(TEST_UUID in rset)
        self.assertEqual(rset[TEST_UUID].msg, msg1)
        self.assertEqual(rset.get(TEST_UUID), rset[TEST_UUID])
        self.assertFalse(DIFF_UUID in rset)
        self.assertIsNone(rset.get(DIFF_UUID))
        self.assertEqual(rset.get(DIFF_UUID, 10), 10)
        self.assertTrue(rset == RequestSet([msg1], RQR_UUID))
        self.assertTrue(rset == RequestSet([msg1], RQR_UUID,
                                           contents=ResourceReply))
        rs2 = copy.deepcopy(rset)
        self.assertTrue(rset == rs2)

        rset_str = """requester_id: 01234567-89ab-cdef-0123-456789abcdef
requests:
  id: 01234567-89ab-cdef-fedc-ba9876543210
    priority: 0
    resources: 
      linux.precise.ros.segbot.*/test_rapp
    status: 0"""
        self.assertEqual(str(rset), rset_str)

        sch_msg = SchedulerRequests(requester=unique_id.toMsg(RQR_UUID),
                                    requests=[msg1])
        self.assertEqual(rset.to_msg(stamp=rospy.Time()), sch_msg)

    def test_two_request_set(self):
        msg1 = Request(id=unique_id.toMsg(TEST_UUID),
                       resources=[TEST_WILDCARD])
        msg2 = Request(id=unique_id.toMsg(DIFF_UUID),
                       resources=[TEST_RESOURCE])
        rset = RequestSet([msg1, msg2], RQR_UUID)
        self.assertEqual(len(rset), 2)
        self.assertTrue(TEST_UUID in rset)
        self.assertTrue(DIFF_UUID in rset)
        self.assertEqual(rset[TEST_UUID].msg, msg1)
        self.assertEqual(rset[DIFF_UUID].msg, msg2)
        self.assertEqual(rset.get(TEST_UUID), rset[TEST_UUID])
        self.assertEqual(rset.get(DIFF_UUID), rset[DIFF_UUID])
        self.assertTrue(rset == RequestSet([msg1, msg2], RQR_UUID))
        self.assertTrue(rset == RequestSet([msg2, msg1], RQR_UUID))
        self.assertTrue(rset != RequestSet([msg1], RQR_UUID))
        self.assertTrue(rset != RequestSet([msg2], RQR_UUID))
        self.assertTrue(rset != RequestSet([], RQR_UUID))
        rs2 = copy.deepcopy(rset)
        self.assertTrue(rset == rs2)
        self.assertFalse(rset != rs2)
        rs2[TEST_UUID]._update_status(Request.GRANTED)
        self.assertTrue(rset != rs2)
        self.assertFalse(rset == rs2)

    def test_empty_merge(self):
        msg1 = Request(id=unique_id.toMsg(TEST_UUID),
                       resources=[TEST_WILDCARD],
                       status=Request.NEW)
        rset = RequestSet([msg1], RQR_UUID)
        self.assertEqual(len(rset), 1)
        self.assertTrue(TEST_UUID in rset)
        sch_msg = SchedulerRequests(requester=unique_id.toMsg(RQR_UUID),
                                    requests=[msg1])
        self.assertEqual(rset.to_msg(stamp=rospy.Time()), sch_msg)

        # merge an empty request set: rset should remain the same
        rset.merge(RequestSet([], RQR_UUID, contents=ResourceReply))
        self.assertEqual(len(rset), 1)
        self.assertTrue(TEST_UUID in rset)
        self.assertEqual(rset.to_msg(stamp=rospy.Time()), sch_msg)

    def test_freed_merge(self):
        msg1 = Request(id=unique_id.toMsg(TEST_UUID),
                       resources=[TEST_WILDCARD],
                       status=Request.RELEASED)
        rset = RequestSet([msg1], RQR_UUID, contents=ResourceReply)
        self.assertEqual(len(rset), 1)
        self.assertTrue(TEST_UUID in rset)
        sch_msg = SchedulerRequests(requester=unique_id.toMsg(RQR_UUID),
                                    requests=[msg1])
        self.assertEqual(rset.to_msg(stamp=rospy.Time()), sch_msg)

        # merge an empty request set: TEST_UUID should be deleted
        empty_rset = RequestSet([], RQR_UUID)
        rset.merge(empty_rset)
        self.assertEqual(len(rset), 0)
        self.assertFalse(TEST_UUID in rset)
        self.assertNotEqual(rset.to_msg(stamp=rospy.Time()), sch_msg)
        self.assertEqual(rset, empty_rset)

    def test_released_merge(self):
        msg1 = Request(id=unique_id.toMsg(TEST_UUID),
                       resources=[TEST_WILDCARD],
                       status=Request.RELEASING)
        rset = RequestSet([msg1], RQR_UUID)
        self.assertEqual(len(rset), 1)
        self.assertTrue(TEST_UUID in rset)
        sch_msg = SchedulerRequests(requester=unique_id.toMsg(RQR_UUID),
                                    requests=[msg1])
        self.assertEqual(rset.to_msg(stamp=rospy.Time()), sch_msg)

        # merge a released request: TEST_UUID should be deleted
        msg2 = Request(id=unique_id.toMsg(TEST_UUID),
                       resources=[TEST_WILDCARD],
                       status=Request.RELEASED)
        rel_rset = RequestSet([msg2], RQR_UUID, contents=ResourceReply)
        rset.merge(rel_rset)
        self.assertEqual(len(rset), 0)
        self.assertFalse(TEST_UUID in rset)
        self.assertEqual(rset, RequestSet([], RQR_UUID,
                                          contents=ResourceReply))
        self.assertNotEqual(rset.to_msg(stamp=rospy.Time()), sch_msg)
        self.assertNotEqual(rset, rel_rset)

    def test_single_merge(self):
        msg1 = Request(id=unique_id.toMsg(TEST_UUID),
                       resources=[TEST_WILDCARD],
                       status=Request.NEW)
        rset = RequestSet([msg1], RQR_UUID)
        self.assertEqual(rset[TEST_UUID].msg.status, Request.NEW)
        self.assertEqual(rset[TEST_UUID].msg.resources, [TEST_WILDCARD])
        self.assertEqual(rset[TEST_UUID].msg.id, unique_id.toMsg(TEST_UUID))

        # merge an updated request set: resource list should change
        msg2 = Request(id=unique_id.toMsg(TEST_UUID),
                       resources=[TEST_RESOURCE],
                       status=Request.GRANTED)
        rset.merge(RequestSet([msg2], RQR_UUID, contents=ResourceReply))
        self.assertEqual(len(rset), 1)
        self.assertTrue(TEST_UUID in rset)
        self.assertEqual(rset[TEST_UUID].msg.status, Request.GRANTED)
        self.assertEqual(rset[TEST_UUID].msg.resources, [TEST_RESOURCE])
        sch_msg = SchedulerRequests(requester=unique_id.toMsg(RQR_UUID),
                                    requests=[msg2])
        self.assertEqual(rset.to_msg(stamp=rospy.Time()), sch_msg)

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('scheduler_request_transitions',
                    'test_transitions',
                    TestTransitions)
