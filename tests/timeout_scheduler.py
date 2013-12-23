#!/usr/bin/env python
""" Scheduler for requester timeout testing. """

# enable some python3 compatibility options:
from __future__ import absolute_import, print_function, unicode_literals

import unittest
from collections import deque
import rospy
from scheduler_msgs.msg import Resource
from rocon_scheduler_requests.transitions import Request
from rocon_scheduler_requests import Scheduler, TransitionError


class TestTimeoutScheduler(unittest.TestCase):

    def test_timeout_scheduler(self):
        """ Initialize ROCON scheduler node for timeout test. """
        rospy.init_node("timeout_scheduler")
        self.avail = deque([            # FIFO queue of available robots
            Resource(name='example_rapp',
                     platform_info='linux.precise.ros.turtlebot.roberto'),
            Resource(name='example_rapp',
                     platform_info='linux.precise.ros.turtlebot.marvin'),
            ])
        self.queue = deque()            # FIFO queue of waiting requests
        self.seen_requester = False
        self.timer = rospy.Timer(rospy.Duration(2.0), self.check_finished)
        self.sch = Scheduler(self.callback, frequency=1.0)
        rospy.spin()

    def allocate(self, requester_id, rq):
        """ Allocate requested resource, if available.

        :returns: True if this request handled; False if no longer valid
        """
        if len(self.avail) > 0:         # resources available?
            resource = self.avail.popleft()
            try:
                rq.grant([resource])
                rospy.loginfo('Request granted: ' + str(rq.get_uuid()))
            except TransitionError:
                # put resource back at the front of the queue
                self.avail.appendleft(resource)
                return False
        else:                           # nothing right now
            self.queue.append((requester_id, rq))
            try:
                rq.wait(reason=Request.BUSY)
                rospy.loginfo('Request queued: ' + str(rq.get_uuid()))
            except TransitionError:
                return False
        try:                            # try to notify requester
            self.sch.notify(requester_id)
        except KeyError:                # requester missing
            return False
        return True

    def callback(self, rset):
        """ Scheduler request callback. """
        rospy.logdebug('scheduler callback:')
        for rq in rset.values():
            rospy.logdebug('  ' + str(rq))
            if rq.msg.status == Request.NEW:
                self.allocate(rset.requester_id, rq)
            elif rq.msg.status == Request.CANCELING:
                self.free(rset.requester_id, rq)

    def check_finished(self, event):
        """ Timer event handler:

        Stops test after at least one requester has come and gone.
        """
        if self.seen_requester:
            if len(self.sch.requesters) == 0:
                rospy.loginfo('requester connection lost')
                rospy.signal_shutdown('test completed.')
        else:
            if len(self.sch.requesters) > 0:
                self.seen_requester = True

    def free(self, requester_id, rq):
        """ Free the resource allocated for this request. """
        self.avail.append(rq.msg.resources[0])
        rospy.loginfo('Request canceled: ' + str(rq.get_uuid()))
        rq.close()
        if len(self.queue) > 0:
            next_rqr, next_rq = self.queue.popleft()
            self.allocate(next_rqr, next_rq)

if __name__ == '__main__':
    import rostest
    rostest.rosrun('rocon_scheduler_requests',
                   'timeout_scheduler', TestTimeoutScheduler)
