#!/usr/bin/env python

# dummy scheduler node for requester testing

# enable some python3 compatibility options:
from __future__ import absolute_import, print_function, unicode_literals

import rospy
import scheduler_request_manager.scheduler as scheduler

if __name__ == '__main__':

    rospy.init_node("dummy_scheduler")

    # Expect allocation requests at 1Hz frequency, just for testing.
    sch = scheduler.Scheduler(frequency=1.0)

    # spin in the main thread: required for message callbacks
    rospy.spin()
