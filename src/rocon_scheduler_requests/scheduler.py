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

Python interface for ROCON schedulers handling resource requests.

This module provides a relatively simple API, not requiring detailed
knowledge of scheduler request state transitions.

.. _`uuid_msgs/UniqueID`:
     http://ros.org/doc/api/uuid_msgs/html/msg/UniqueID.html
.. _UUID: http://en.wikipedia.org/wiki/Uuid

"""

# enable some python3 compatibility options:
from __future__ import absolute_import, print_function, unicode_literals

import rospy
import unique_id

# ROS messages
from scheduler_msgs.msg import Request
from scheduler_msgs.msg import SchedulerRequests

# internal modules
from . import common
from . import transitions


class _RequesterStatus:
    """
    This class tracks the status of all resource requests made by a
    single requester.  It subscribes to the requester feedback topic,
    and provides updated information when appropriate.

    :param sched: (:class:`.Scheduler`) Scheduler object with which
        this requester is connected.

    :param msg: (scheduler_msgs/SchedulerRequests) Initial resource
        allocation requests.

    """

    def __init__(self, sched, msg):
        """ Constructor. """

        self.sched = sched
        """ Scheduler for this requester. """
        self.requester_id = unique_id.fromMsg(msg.requester)
        """ :class:`uuid.UUID` of this requester. """
        self.rset = transitions.RequestSet([], self.requester_id,
                                           priority=msg.priority,
                                           replies=True)
        """ All current scheduler replies to this requester. """

        self.feedback_topic = common.feedback_topic(self.requester_id,
                                                    self.sched.topic)
        rospy.loginfo('requester feedback topic: ' + self.feedback_topic)

        self.pub = rospy.Publisher(self.feedback_topic,
                                   SchedulerRequests)
        self.feedback_msg = SchedulerRequests(requester=msg.requester,
                                              priority=msg.priority)

        self.update(msg)        # set initial status

    def _send_feedback(self):
        """ Build feedback message and send it to the requester. """
        self.feedback_msg.header.stamp = rospy.Time.now()
        self.feedback_msg.requests = self.rset.list_requests()
        self.pub.publish(self.feedback_msg)

    def update(self, msg):
        """ Update requester status.

        :param msg: Latest resource allocation request.
        :type msg: scheduler_msgs/SchedulerRequests

        """
        # Make a new RequestSet from this message
        requests = None
        requests = msg.requests
        new_rset = transitions.RequestSet(requests,
                                          self.requester_id,
                                          priority=msg.priority,
                                          replies=False)
        self.rset.merge(new_rset)
        self.sched.callback(self.rset)
        self._send_feedback()   # notify the requester

    def timeout(self, limit, event):
        """ Check for requester timeout.

        :returns: true if time limit exceeded.

        :todo: handle this timeout

        """
        return False            # test scaffolding


class Scheduler:
    """
    This class is used by a ROCON scheduler to manage all the resource
    requests sent by various ROCON services.  It subscribes to the
    ROCON scheduler topic, handling resource requests as they are
    received.

    :param callback: Callback function invoked with the updated
        :class:`.RequestSet` when requests arrive.

    :param frequency: requester heartbeat frequency in Hz.
    :type frequency: float
    :param topic: Topic name for resource allocation requests.
    :type topic: str

    .. describe:: callback(rset)

       :param rset: (:class:`.RequestSet`) The current status of all
           requests for some active requester.

    The *callback* function is expected to iterate over its
    :class:`.RequestSet`, checking the status of every
    :class:`.ResourceReply` it contains, and modify them
    appropriately.

    """

    def __init__(self, callback,
                 frequency=common.HEARTBEAT_HZ,
                 topic=common.SCHEDULER_TOPIC):
        """ Constructor. """
        self.callback = callback
        """ Callback function for request updates. """
        self.requesters = {}
        """ Dictionary of active requesters and their requests. """
        self.topic = topic
        """ Scheduler request topic name. """
        rospy.loginfo('scheduler request topic: ' + self.topic)
        self.sub = rospy.Subscriber(self.topic,
                                    SchedulerRequests,
                                    self._allocate_resources)
        self.duration = rospy.Duration(1.0 / frequency)
        self.time_limit = self.duration * 4.0
        self.timer = rospy.Timer(self.duration, self._watchdog)

    def _allocate_resources(self, msg):
        """ Scheduler resource allocation message handler. """
        rqr_id = unique_id.fromMsg(msg.requester)
        rqr = self.requesters.get(rqr_id)
        if rqr:                 # known requester?
            rqr.update(msg)
        else:                   # new requester
            self.requesters[rqr_id] = _RequesterStatus(self, msg)

    def _watchdog(self, event):
        """ Scheduler request watchdog timer handler. """
        # Must iterate over a copy of the dictionary items, because
        # some may be deleted inside the loop.
        for rqr_id, rqr in self.requesters.items():
            if rqr.timeout(self.time_limit, event):
                del self.requesters[rqr_id]
