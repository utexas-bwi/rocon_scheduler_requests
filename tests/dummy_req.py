#!/usr/bin/env python

# enable some python3 compatibility options:
from __future__ import absolute_import, print_function

import rospy
import uuid
import unique_id
import scheduler_request_manager.requester as requester
import scheduler_request_manager.transitions as transitions

# ROS messages
from rocon_std_msgs.msg import PlatformInfo
from scheduler_msgs.msg import Request

# Constants
TEST_UUID_HEX = '0123456789abcdef0123456789abcdef'
TEST_UUID = uuid.UUID(hex=TEST_UUID_HEX)
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

rid = None                      # UUID for Resource Request

def feedback(rset):
    if rid:
        rq = rset[rid]
        if rq:
            if rq.msg.status == Request.WAITING:
                print('Request queued')
            elif rq.msg.status == Request.GRANTED:
                print('Request granted')
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
    rid = rqr.new_request(TEST_WILDCARD)

    # Spin in the main thread: required for message callbacks.
    rospy.spin()
