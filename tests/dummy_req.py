#!/usr/bin/env python

# enable some python3 compatibility options:
from __future__ import absolute_import, print_function

import rospy
import uuid
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
    #print(str(rset))
    if rid:
        rq = rset.get(rid)
        if rq:
            if rq.msg.status == Request.WAITING:
                print('Request queued: ' + str(rq.get_uuid()))
            elif rq.msg.status == Request.GRANTED:
                print('Request granted: ' + str(rq.get_uuid()))
                rq.release()
            elif rq.msg.status == Request.RELEASING:
                print('Request released: ' + str(rq.get_uuid()))
        else:
            print('Request no longer active: ' + str(rid))

if __name__ == '__main__':

    rospy.init_node("dummy_requester")

    # Set heartbeat frequency to 1Hz, speeds up testing.
    rqr = requester.Requester(feedback, uuid=TEST_UUID, frequency=1.0)
    rqr.send_requests()         # send empty request message

    rospy.sleep(2.0)            # wait a while

    # Make a new request using a wildcard resource, and send it to the scheduler.
    rid = rqr.new_request([TEST_WILDCARD])
    rqr.send_requests()

    # Spin in the main thread: required for message callbacks.
    rospy.spin()
