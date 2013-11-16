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
.. module:: requester

Python interface for rocon services making scheduler requests.

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
from . import common

class Requester:
    """
    This class is used by a rocon service to handle its resource
    requests.

    :param uuid: UUID_ of this requester. If None provided, a random
                 uuid will be assigned.
    :type uuid: Standard Python :class:`uuid.UUID` object.
    :param frequency: requester heartbeat frequency in Hz.
    :type frequency: float
    :param topic: Topic name for allocating resources.
    :type topic: str

    """

    def __init__(self, uuid=None,
                 frequency=common.HEARTBEAT_HZ,
                 topic=common.SCHEDULER_TOPIC):
        """Constructor.

        Initializes the :class:`Requester`, subscribes to its own
        scheduler feedback topic and advertises the rocon scheduler
        topic.
        """
        if uuid is None:
            uuid = unique_id.fromRandom()
        self.requester_id = uuid
        self.pub_topic = topic
        self.sub_topic = common.feedback_topic(uuid, topic)
        rospy.loginfo('Rocon resource requester topic: ' + self.sub_topic)
        self.sub = rospy.Subscriber(self.sub_topic,
                                    SchedulerFeedback,
                                    self._feedback)
        self.alloc = AllocateResources()
        self.alloc.requester = unique_id.toMsg(self.requester_id)
        self.pub = rospy.Publisher(self.pub_topic, AllocateResources)
        self.timer = rospy.Timer(rospy.Duration(1.0 / frequency),
                                 self._heartbeat)

    def _feedback(self, msg):
        """ Scheduler feedback message handler."""
        pass                    # test scaffolding

    def _heartbeat(self, event):
        """ Scheduler request heartbeat timer handler.

        Publishes all current allocation requests to the scheduler.
        """
        rospy.loginfo('Rocon resource requester heartbeat')
        print('Rocon resource requester heartbeat')
        self.alloc.header.stamp = event.current_real
        self.pub.publish(self.alloc)
