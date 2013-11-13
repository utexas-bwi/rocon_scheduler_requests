# Software License Agreement (BSD License)
#
# Copyright (C) 2013, Jack O'Quin
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the author nor of other contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
.. module:: scheduler

Python interface for rocon schedulers handling resource requests.

This module provides a relatively simple API, not requiring detailed
knowledge of scheduler request state transitions.

.. _`uuid_msgs/UniqueID`: http://ros.org/doc/api/uuid_msgs/html/msg/UniqueID.html
.. _UUID: http://en.wikipedia.org/wiki/Uuid

"""

# enable some python3 compatibility options:
from __future__ import absolute_import, print_function, unicode_literals

import rospy
import unique_id
from scheduler_msgs.msg import AllocateResources
from scheduler_msgs.msg import Request
from scheduler_msgs.msg import SchedulerFeedback

SCHEDULER_TOPIC = "rocon_scheduler"

class Scheduler:
    """
    :class:`Scheduler` manages the resource request messages sent by
    various rocon services.

    """

    def __init__(self):
        """Constructor.

        Initializes the :class:`Scheduler` and subscribes to the
        **/scheduler_requests** topic.
        """
        self.topic_name = SCHEDULER_TOPIC
        rospy.loginfo('Rocon scheduler request topic: ' + self.topic_name)
        self.sub = rospy.Subscriber(self.topic_name,
                                    AllocateResources,
                                    self.allocate_resources)
        self.alloc = AllocateResources()
        self.timer = rospy.Timer(rospy.Duration(1.0), self.watchdog)

    def allocate_resources(self, msg):
        """ Scheduler resource allocation message handler.
        """
        # test scaffolding
        rospy.loginfo('Rocon scheduler request: \n' + msg)

    def watchdog(self, event):
        """ Scheduler request watchdog timer handler.
        """
        # test scaffolding
        rospy.loginfo('Rocon resource watchdog heartbeat')
