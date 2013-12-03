#!/usr/bin/env python

# enable some python3 compatibility options:
from __future__ import absolute_import, print_function

import rospy
import uuid
import unique_id
import rocon_scheduler_requests.requester as requester
import rocon_scheduler_requests.transitions as transitions

# ROS messages
from scheduler_msgs.msg import Request
from scheduler_msgs.msg import Resource

# Constants
TEST_UUID_HEX = '0123456789abcdef0123456789abcdef'
TEST_UUID = uuid.UUID(hex=TEST_UUID_HEX)
TEST_RAPP = 'test_rapp'
TEST_RESOURCE = Resource(name=TEST_RAPP,
                         platform_info='linux.precise.ros.segbot.roberto')
TEST_WILDCARD = Resource(name=TEST_RAPP,
                         platform_info='linux.precise.ros.segbot.*')

rid = None                      # UUID for Resource Request

def feedback(rset):
    print(str(rset))
    if rid:
        rq = rset[rid]
        if rq:
            if rq.msg.status == Request.WAITING:
                print('Request queued')
            elif rq.msg.status == Request.GRANTED:
                print('Request granted, resource: ' + rq.str_resources())
                rset[rid].release()
            elif rq.msg.status == Request.RELEASING:
                print('Request released')
        else:
            print('Request freed.')

if __name__ == '__main__':

    rospy.init_node("dummy_requester")

    # Repeat allocation requests at 1Hz frequency, speeds up testing.
    rqr = requester.Requester(feedback, uuid=TEST_UUID, frequency=1.0)

    # Make a new request using a wildcard resource.
    rid = rqr.new_request([TEST_WILDCARD])

    # Spin in the main thread: required for message callbacks.
    rospy.spin()
