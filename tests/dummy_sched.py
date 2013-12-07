#!/usr/bin/env python

# dummy scheduler node for requester testing

# enable some python3 compatibility options:
from __future__ import absolute_import, print_function, unicode_literals

import rospy
import unique_id
from scheduler_msgs.msg import Request
from scheduler_msgs.msg import Resource
import rocon_scheduler_requests.scheduler as scheduler

# Constants
TEST_RESOURCE = Resource(name='test_rapp',
                         platform_info='linux.precise.ros.segbot.roberto')

def callback(rset):
    """ Scheduler request callback. """
    # :todo: make new requests wait, then grant after a second
    #print(str(rset))
    for rq in rset.values():
        if rq.msg.status == Request.NEW:
            rq.grant([TEST_RESOURCE])
            print('Request granted: ' + str(unique_id.fromMsg(rq.msg.id)))
        elif rq.msg.status == Request.RELEASING:
            rq.free()
            print('Request released: ' + str(unique_id.fromMsg(rq.msg.id)))

if __name__ == '__main__':

    rospy.init_node("dummy_scheduler")

    # Expect allocation requests at 1Hz frequency, just for testing.
    sch = scheduler.Scheduler(callback, frequency=1.0)

    # spin in the main thread: required for message callbacks
    rospy.spin()
