#!/usr/bin/env python
""" Scheduler usage example. """
import collections
import rospy
from scheduler_msgs.msg import Request, Resource
from rocon_scheduler_requests.scheduler import Scheduler


class ExampleScheduler:

    def __init__(self):
        rospy.init_node("example_scheduler")
        self.avail = [                  # list of available robots
            Resource(name='example_rapp',
                     platform_info='linux.precise.ros.turtlebot.roberto'),
            Resource(name='example_rapp',
                     platform_info='linux.precise.ros.turtlebot.marvin'),
            ]
        self.queue = collections.deque()
        self.sch = Scheduler(self.callback)
        rospy.spin()

    def allocate(self, requester_id, rq):
        """ Allocate requested resource, if available. """
        if len(self.avail) > 0:         # resources available?
            rq.grant([self.avail.pop()])
            rospy.loginfo('Request granted: ' + str(rq.get_uuid()))
        else:                           # nothing right now
            self.queue.append((requester_id, rq))
            rq.wait()
            rospy.loginfo('Request queued: ' + str(rq.get_uuid()))
        try:                            # try to notify requester
            self.sch.notify(requester_id)
        except KeyError:                # requester no longer there
            pass

    def callback(self, rset):
        """ Scheduler request callback: queue new requests until available. """
        for rq in rset.values():
            if rq.msg.status == Request.NEW:
                self.allocate(rset.requester_id, rq)
            elif rq.msg.status == Request.RELEASING:
                self.free(rset.requester_id, rq)

    def free(self, requester_id, rq):
        """ Free the resource allocated for this request. """
        self.avail.append(rq.msg.resources[0])
        rospy.loginfo('Request canceled: ' + str(rq.get_uuid()))
        rq.free()
        if len(self.queue) > 0:
            pair = popleft(self.avail)
            self.allocate(pair[0], pair[1])

if __name__ == '__main__':
    node = ExampleScheduler()
