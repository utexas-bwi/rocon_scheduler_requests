#!/usr/bin/env python
""" Scheduler usage example. """
from collections import deque
import rospy
from scheduler_msgs.msg import Resource
from rocon_scheduler_requests.transitions import Request
from rocon_scheduler_requests import Scheduler


class ExampleScheduler:

    def __init__(self):
        rospy.init_node("example_scheduler")
        self.avail = deque([            # FIFO queue of available robots
            Resource(name='example_rapp',
                     platform_info='linux.precise.ros.turtlebot.roberto'),
            Resource(name='example_rapp',
                     platform_info='linux.precise.ros.turtlebot.marvin'),
            ])
        self.queue = deque()            # FIFO queue of waiting requests
        self.sch = Scheduler(self.callback)
        rospy.spin()

    def allocate(self, requester_id, rq):
        """ Allocate requested resource, if available. """
        if len(self.avail) > 0:         # resources available?
            rq.grant([self.avail.popleft()])
            rospy.loginfo('Request granted: ' + str(rq.get_uuid()))
        else:                           # nothing right now
            self.queue.append((requester_id, rq))
            rq.wait(reason=Request.BUSY)
            rospy.loginfo('Request queued: ' + str(rq.get_uuid()))
        try:                            # try to notify requester
            self.sch.notify(requester_id)
        except KeyError:                # requester no longer there
            pass

    def callback(self, rset):
        """ Scheduler request callback. """
        rospy.logdebug('scheduler callback:')
        for rq in rset.values():
            rospy.logdebug('  ' + str(rq))
            if rq.msg.status == Request.NEW:
                self.allocate(rset.requester_id, rq)
            elif rq.msg.status == Request.CANCELING:
                self.free(rset.requester_id, rq)

    def free(self, requester_id, rq):
        """ Free the resource allocated for this request. """
        self.avail.append(rq.msg.resources[0])
        rospy.loginfo('Request canceled: ' + str(rq.get_uuid()))
        rq.close()
        if len(self.queue) > 0:
            pair = self.queue.popleft()
            self.allocate(pair[0], pair[1])

if __name__ == '__main__':
    node = ExampleScheduler()
