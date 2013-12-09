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

# Global variables
queued_request = None           # pending request
queued_requester = None         # corresponding requester ID
sch = None                      # scheduler object
timer = None                    # timer request object

def allocate(event):
    """ Timer handler: simulated resource availability event. """
    global queued_request
    global queued_requester
    global sch
    if queued_request:
        queued_request.grant([TEST_RESOURCE])
        print('Request granted: ' + str(queued_request.get_uuid()))
        queued_request = None
        sch.notify(queued_requester)

def queue(requester_id, request):
    """ Queue timer request to simulate delayed resource availability. """
    global queued_request
    global queued_requester
    global timer
    queued_requester = requester_id
    queued_request = request
    timer = rospy.Timer(rospy.Duration(1.0), allocate, oneshot=True)

def callback(rset):
    """ Scheduler request callback. """
    # make new requests wait, then grant after a second
    for rq in rset.values():
        if rq.msg.status == Request.NEW:
            rq.wait()
            queue(rset.requester_id, rq)
            print('Request queued: ' + str(rq.get_uuid()))
        elif rq.msg.status == Request.RELEASING:
            rq.free()
            print('Request released: ' + str(rq.get_uuid()))

if __name__ == '__main__':

    rospy.init_node("dummy_scheduler")

    # Expect allocation requests at 1Hz frequency, just for testing.
    sch = scheduler.Scheduler(callback, frequency=1.0)

    # spin in the main thread: required for message callbacks
    rospy.spin()
