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

# ROS messages
from scheduler_msgs.msg import AllocateResources
from scheduler_msgs.msg import Request
from scheduler_msgs.msg import SchedulerFeedback

# internal modules
from . import common
from . import transitions

class Requests:
    """
    This class tracks the status of all resource requests made by a
    single requester.

    :param msg: Initial resource allocation request.
    :type msg: scheduler_msgs/AllocateResources
    :param topic: Topic name for resource allocation requests.
    :type topic: str

    """

    def __init__(self, msg, topic):
        """ Constructor.

        Initializes the :class:`Requests` and subscribes to the
        requester feedback topic.
        """
        requester_id = unique_id.fromMsg(msg.requester)
        self.feedback_topic = common.feedback_topic(requester_id, topic)
        rospy.loginfo('Rocon scheduler feedback topic: ' + self.feedback_topic)
        self.pub = rospy.Publisher(self.feedback_topic, SchedulerFeedback)

        # set initial status using this message
        self.feedback_msg = SchedulerFeedback(requester=msg.requester,
                                              priority=msg.priority)
        self.resources = {}
        self.update(msg)

    def _send_feedback(self):
        """ Build feedback message and send it to the requester. """
        self.feedback_msg.header.stamp = rospy.Time.now()
        self.pub.publish(self.feedback_msg)

    def update(self, msg):
        """ Update requester status.

        :param msg: Latest resource allocation request.
        :type msg: scheduler_msgs/AllocateResources

        """
        # Add any new requests to the dictionary.
        for res in msg.resources:
            rid = unique_id.fromMsg(res.id)
            if rid not in self.resources: # new request?
                self.resources[rid] = transitions.Request(res)

        # Update status of all requests.  Must iterate over a copy of
        # the dictionary items, because some may be deleted inside the
        # loop.
        for rid, rq in self.resources.items():
            rq.update(msg)
            #if rq.status = ZOMBIE:
            #    del self.resources[rid]

        self._send_feedback()

    def timeout(self, limit, event):
        """ Check for requester timeout.

        :returns: true if time limit exceeded.

        """
        return False            # test scaffolding

class Scheduler:
    """
    This class is used by a rocon scheduler to manage all the resource
    requests sent by various rocon services.

    :param frequency: requester heartbeat frequency in Hz.
    :type frequency: float
    :param topic: Topic name for resource allocation requests.
    :type topic: str

    """

    def __init__(self,
                 frequency=common.HEARTBEAT_HZ,
                 topic=common.SCHEDULER_TOPIC):
        """Constructor.

        Initializes the :class:`Scheduler` and subscribes to the rocon
        scheduler topic.
        """
        self.requests = {}      # dict of requesters and their requests
        self.topic = topic
        rospy.loginfo('Rocon scheduler request topic: ' + self.topic)
        self.sub = rospy.Subscriber(self.topic,
                                    AllocateResources,
                                    self._allocate_resources)
        self.duration = rospy.Duration(1.0 / frequency)
        self.time_limit = self.duration * 4.0
        self.timer = rospy.Timer(self.duration, self._watchdog)

    def _allocate_resources(self, msg):
        """ Scheduler resource allocation message handler. """
        # test scaffolding
        rospy.loginfo('Rocon scheduler request: \n' + str(msg))
        requester_id = unique_id.fromMsg(msg.requester)
        rqr = self.requests.get(requester_id)
        if rqr:                 # known requester?
            rqr.update(msg)
        else:                   # new requester
            self.requests[requester_id] = Requests(msg, self.topic)

    def _watchdog(self, event):
        """ Scheduler request watchdog timer handler. """
        # Must iterate over a copy of the dictionary items, because
        # some may be deleted inside the loop.
        for requester, rq in self.requests.items():
            if rq.timeout(self.time_limit, event):
                del self.requests[requester]
