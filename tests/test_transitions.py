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

    ####################
    # utility methods
    ####################
    def assert_invalid(self, request_type, old_status,
                       operation, exception, *args):
        """
        Assert that *request_type* with *old_status* rejects named
        *operation*, raising *exception*.
        """
        rq = request_type(Request(id=unique_id.toMsg(TEST_UUID),
                                  resources=[TEST_WILDCARD],
                                  status=old_status))
        op_method = getattr(rq, operation)
        self.assertRaises(exception, op_method, *args)

    def assert_valid(self, request_type, old_status,
                     operation, new_status, *args):
        """
        Assert that *request_type* with *old_status* accepts named
        *operation*, yielding *new_status*.

        :returns: request contents after the *operation*.
        """
        rq = request_type(Request(id=unique_id.toMsg(TEST_UUID),
                                  resources=[TEST_WILDCARD],
                                  status=old_status))
        getattr(rq, operation)(*args)
        self.assertEqual(rq.msg.status, new_status)
        return rq

    ####################
    # request tests
    ####################
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

    def test_cancel(self):
        # should be valid in every state:
        self.assert_valid(ResourceRequest, Request.CANCELING,
                          'cancel', Request.CANCELING)
        self.assert_valid(ResourceRequest, Request.CLOSED,
                          'cancel', Request.CLOSED)
        self.assert_valid(ResourceRequest, Request.GRANTED,
                          'cancel', Request.CANCELING)
        self.assert_valid(ResourceRequest, Request.NEW,
                          'cancel', Request.CANCELING)
        self.assert_valid(ResourceRequest, Request.PREEMPTING,
                          'cancel', Request.CANCELING)
        self.assert_valid(ResourceRequest, Request.RESERVED,
                          'cancel', Request.CANCELING)
        self.assert_valid(ResourceRequest, Request.WAITING,
                          'cancel', Request.CANCELING)

    def test_close(self):
        self.assert_valid(ResourceReply, Request.CANCELING,
                          'close', Request.CLOSED)
        self.assert_invalid(ResourceReply, Request.CLOSED,
                            'close', TransitionError)
        self.assert_invalid(ResourceReply, Request.GRANTED,
                            'close', TransitionError)
        self.assert_invalid(ResourceReply, Request.NEW,
                            'close', TransitionError)
        self.assert_valid(ResourceReply, Request.PREEMPTING,
                          'close', Request.CLOSED)
        self.assert_invalid(ResourceReply, Request.RESERVED,
                            'close', TransitionError)
        self.assert_invalid(ResourceReply, Request.WAITING,
                            'close', TransitionError)

    def test_grant(self):
        self.assert_invalid(ResourceReply, Request.CANCELING,
                            'grant', TransitionError, [TEST_RESOURCE])
        self.assert_invalid(ResourceReply, Request.CLOSED,
                            'grant', TransitionError, [TEST_RESOURCE])
        self.assert_invalid(ResourceReply, Request.GRANTED,
                            'grant', TransitionError, [TEST_RESOURCE])
        rq = self.assert_valid(ResourceReply, Request.NEW,
                               'grant', Request.GRANTED, [TEST_RESOURCE])
        self.assertEqual(rq.msg.resources, [TEST_RESOURCE])
        if HAVE_REASON:
            self.assertEqual(rq.msg.reason, Request.NONE)
        self.assert_valid(ResourceReply, Request.RESERVED,
                          'grant', Request.GRANTED, [TEST_RESOURCE])
        self.assert_invalid(ResourceReply, Request.PREEMPTING,
                               'grant', TransitionError, [TEST_RESOURCE])
        self.assert_valid(ResourceReply, Request.WAITING,
                          'grant', Request.GRANTED, [TEST_RESOURCE])

    def test_preempt(self):
        # valid in every state, but only affects GRANTED requests
        self.assert_valid(ResourceReply, Request.CANCELING,
                          'preempt', Request.CANCELING)
        rq = self.assert_valid(ResourceReply, Request.CLOSED,
                               'preempt', Request.CLOSED,
                               Request.PREEMPTED)
        if HAVE_REASON:
            self.assertNotEqual(rq.msg.reason, Request.PREEMPTED)
            self.assertEqual(rq.msg.reason, Request.NONE)
        rq = self.assert_valid(ResourceReply, Request.GRANTED,
                               'preempt', Request.PREEMPTING,
                               Request.PREEMPTED)
        if HAVE_REASON:
            self.assertEqual(rq.msg.reason, Request.PREEMPTED)
        self.assert_valid(ResourceReply, Request.NEW,
                          'preempt', Request.NEW)
        self.assert_valid(ResourceReply, Request.PREEMPTING,
                          'preempt', Request.PREEMPTING)
        self.assert_valid(ResourceReply, Request.RESERVED,
                          'preempt', Request.RESERVED)
        self.assert_valid(ResourceReply, Request.WAITING,
                          'preempt', Request.WAITING)

    def test_validate(self):
        rq1 = ResourceRequest(Request(id=unique_id.toMsg(TEST_UUID),
                                      resources=[TEST_RESOURCE],
                                      status=Request.NEW))
        self.assertTrue(rq1._validate(Request.GRANTED))
        self.assertTrue(rq1._validate(Request.PREEMPTING))
        self.assertFalse(rq1._validate(Request.CLOSED))


    def test_wait(self):
        self.assert_invalid(ResourceReply, Request.CANCELING,
                            'wait', TransitionError)
        self.assert_invalid(ResourceReply, Request.CLOSED,
                            'wait', TransitionError)
        self.assert_invalid(ResourceReply, Request.GRANTED,
                            'wait', TransitionError)
        rq = self.assert_valid(ResourceReply, Request.NEW,
                               'wait', Request.WAITING, Request.BUSY)
        if HAVE_REASON:
            self.assertEqual(rq.msg.reason, Request.BUSY)
        self.assert_invalid(ResourceReply, Request.PREEMPTING,
                            'wait', TransitionError)
        rq = self.assert_valid(ResourceReply, Request.RESERVED,
                               'wait', Request.WAITING, Request.UNAVAILABLE)
        if HAVE_REASON:
            self.assertEqual(rq.msg.reason, Request.UNAVAILABLE)
        rq = self.assert_invalid(ResourceReply, Request.WAITING,
                                 'wait', TransitionError)


class TestRequestSets(unittest.TestCase):
    """Unit tests for scheduler request state transitions.

    These tests do not require a running ROS core.
    """

    ####################
    # request set tests
    ####################

    def test_empty_request_set(self):
        self.assertRaises(TypeError, RequestSet)
        self.assertRaises(TypeError, RequestSet, [])
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
        rs2[TEST_UUID]._transition(EVENT_GRANT)
        self.assertTrue(rset != rs2)
        self.assertFalse(rset == rs2)

    def test_request_set_from_scheduler_requests(self):
        msg1 = Request(id=unique_id.toMsg(TEST_UUID),
                       resources=[TEST_WILDCARD])
        msg2 = Request(id=unique_id.toMsg(DIFF_UUID),
                       resources=[TEST_RESOURCE])
        schreq = SchedulerRequests(requester=unique_id.toMsg(RQR_UUID),
                                   requests=[msg1, msg2])
        rset = RequestSet(schreq)
        self.assertEqual(len(rset), 2)
        self.assertTrue(TEST_UUID in rset)
        self.assertTrue(DIFF_UUID in rset)
        self.assertEqual(rset[TEST_UUID].msg, msg1)
        self.assertEqual(rset[DIFF_UUID].msg, msg2)
        self.assertEqual(rset.get(TEST_UUID), rset[TEST_UUID])
        self.assertEqual(rset.get(DIFF_UUID), rset[DIFF_UUID])
        self.assertTrue(rset == RequestSet(schreq))
        self.assertTrue(rset == RequestSet(schreq, RQR_UUID))
        self.assertTrue(rset != RequestSet([msg1], RQR_UUID))
        self.assertTrue(rset != RequestSet([msg2], RQR_UUID))
        self.assertTrue(rset != RequestSet([], RQR_UUID))
        rs2 = copy.deepcopy(rset)
        self.assertTrue(rset == rs2)
        self.assertFalse(rset != rs2)
        rs2[TEST_UUID]._transition(EVENT_GRANT)
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

    def test_closed_merge(self):
        msg1 = Request(id=unique_id.toMsg(TEST_UUID),
                       resources=[TEST_WILDCARD],
                       status=Request.CLOSED)
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

    def test_canceled_merge(self):
        msg1 = Request(id=unique_id.toMsg(TEST_UUID),
                       resources=[TEST_WILDCARD],
                       status=Request.CANCELING)
        rset = RequestSet([msg1], RQR_UUID)
        self.assertEqual(len(rset), 1)
        self.assertTrue(TEST_UUID in rset)
        sch_msg = SchedulerRequests(requester=unique_id.toMsg(RQR_UUID),
                                    requests=[msg1])
        self.assertEqual(rset.to_msg(stamp=rospy.Time()), sch_msg)

        # merge a canceled request: TEST_UUID should be deleted
        msg2 = Request(id=unique_id.toMsg(TEST_UUID),
                       resources=[TEST_WILDCARD],
                       status=Request.CLOSED)
        rel_rset = RequestSet([msg2], RQR_UUID, contents=ResourceReply)
        rset.merge(rel_rset)
        self.assertEqual(len(rset), 0)
        self.assertFalse(TEST_UUID in rset)
        self.assertEqual(rset, RequestSet([], RQR_UUID,
                                          contents=ResourceReply))
        self.assertNotEqual(rset.to_msg(stamp=rospy.Time()), sch_msg)
        self.assertNotEqual(rset, rel_rset)

    def test_canceled_merge_plus_new_request(self):
        msg1 = Request(id=unique_id.toMsg(TEST_UUID),
                       resources=[TEST_RESOURCE],
                       status=Request.CANCELING)
        msg2 = Request(id=unique_id.toMsg(DIFF_UUID),
                       resources=[TEST_WILDCARD],
                       status=Request.NEW)
        rset = RequestSet([msg1, msg2], RQR_UUID)
        self.assertEqual(len(rset), 2)
        self.assertTrue(TEST_UUID in rset)
        self.assertTrue(DIFF_UUID in rset)
        self.assertEqual(rset[DIFF_UUID].msg.status, Request.NEW)

        # merge a canceled request: TEST_UUID should be deleted, but
        # DIFF_UUID should not
        msg3 = Request(id=unique_id.toMsg(TEST_UUID),
                       resources=[TEST_RESOURCE],
                       status=Request.CLOSED)
        rel_rset = RequestSet([msg3], RQR_UUID)
        rset.merge(rel_rset)
        self.assertEqual(len(rset), 1)
        self.assertTrue(TEST_UUID not in rset)
        self.assertTrue(DIFF_UUID in rset)
        self.assertEqual(rset[DIFF_UUID].msg.status, Request.NEW)

        # make a fresh object like the original msg2 for comparison
        msg4 = Request(id=unique_id.toMsg(DIFF_UUID),
                       resources=[TEST_WILDCARD],
                       status=Request.NEW)
        self.assertEqual(rset, RequestSet([msg4], RQR_UUID))
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
    rosunit.unitrun('rocon_scheduler_requests',
                    'test_transitions',
                    TestTransitions)
    rosunit.unitrun('rocon_scheduler_requests',
                    'test_request_sets',
                    TestRequestSets)
