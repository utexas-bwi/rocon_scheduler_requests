#!/usr/bin/env python

# dummy scheduler node for requester testing

# enable some python3 compatibility options:
from __future__ import absolute_import, print_function, unicode_literals

import rospy
from scheduler_msgs.msg import Request
from scheduler_msgs.msg import Resource
import rocon_scheduler_requests.scheduler as scheduler

# Constants
TEST_RESOURCE = Resource(name='test_rapp',
                         platform_info='linux.precise.ros.segbot.roberto')

def callback(rset):
    """ Scheduler request callback. """
    print(str(rset))
    for rq in rset.values():
        #print(str(rq))
        if rq.msg.status == Request.NEW:
            rq.wait()
            print('Request queued')
        elif rq.msg.status == Request.WAITING:
            rq.grant([TEST_RESOURCE])
            print('Request granted')
        elif rq.msg.status == Request.RELEASING:
            rq.free()
            print('Request released')

if __name__ == '__main__':

    rospy.init_node("dummy_scheduler")

    # Expect allocation requests at 1Hz frequency, just for testing.
    sch = scheduler.Scheduler(callback, frequency=1.0)

    # spin in the main thread: required for message callbacks
    rospy.spin()
