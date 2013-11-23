#!/usr/bin/env python

# enable some python3 compatibility options:
from __future__ import absolute_import, print_function

import rospy
import uuid
import scheduler_request_manager.requester as requester
import scheduler_request_manager.transitions as transitions

TEST_UUID_HEX = '0123456789abcdef0123456789abcdef'
TEST_UUID = uuid.UUID(hex=TEST_UUID_HEX)

def feedback(rset):
    print(str(rset))            # test scaffolding

if __name__ == '__main__':

    rospy.init_node("dummy_requester")

    # Expect allocation requests at 1Hz frequency, just for testing.
    rqr = requester.Requester(feedback, uuid=TEST_UUID, frequency=1.0)

    # spin in the main thread: required for message callbacks
    rospy.spin()
