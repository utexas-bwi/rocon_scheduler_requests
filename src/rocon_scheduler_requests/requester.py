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

Python interface for ROCON services making scheduler requests.

This module provides a relatively simple API, not requiring detailed
knowledge of scheduler request messages or state transitions.

.. _`uuid_msgs/UniqueID`:
     http://ros.org/doc/api/uuid_msgs/html/msg/UniqueID.html
.. _UUID: http://en.wikipedia.org/wiki/Uuid

"""

# enable some python3 compatibility options:
from __future__ import absolute_import, print_function, unicode_literals

import copy

# ROS dependencies
import rospy
import unique_id

# ROS messages
from scheduler_msgs.msg import Request
from scheduler_msgs.msg import SchedulerRequests

# internal modules
from . import common
from . import transitions


class Requester:
    """
    This class is used by a ROCON service to handle its resource
    requests.  When an instance of :class:`.Requester` is created, it
    creates its own scheduler feedback topic and connects to the ROCON
    scheduler topic.

    :param feedback: Callback function invoked with the current
                     :class:`.RequestSet` when feedback arrives.

    :param uuid: UUID_ of this requester. If ``None`` provided, a random
                 UUID will be assigned.
    :type uuid: :class:`uuid.UUID`

    :param priority: default priority for requests from this requester.

    :param topic: Topic name for allocating resources.
    :type topic: str

    :param frequency: requester heartbeat frequency in Hz.  Use the
                      default, except in exceptional situations or for
                      testing.
    :type frequency: float

    As long as the :class:`.Requester` object remains, it will
    periodically send request messages to the scheduler, which will
    provide feedback for them.  Those messages may be empty if no
    requests are outstanding.  The caller-provided ``feedback`` function
    will be invoked each time a feedback message arrives, like this:

    .. describe:: feedback(rset)

       :param rset: The current set of requests including possible
                    updates from the scheduler.
       :type rset: :class:`.RequestSet`

    The ``feedback`` function is expected to iterate over its
    :class:`.RequestSet`, checking the status of every
    :class:`.ResourceRequest` it contains, and modify them
    appropriately.

    """

    def __init__(self, feedback, uuid=None,
                 priority=0,
                 topic=common.SCHEDULER_TOPIC,
                 frequency=common.HEARTBEAT_HZ):
        """ Constructor. """

        if uuid is None:
            uuid = unique_id.fromRandom()
        self.requester_id = uuid
        """ :class:`uuid.UUID` of this requester. """
        self.rset = transitions.RequestSet([], self.requester_id)
        """
        :class:`.RequestSet` containing the current status of every
        :class:`.ResourceRequest` made by this requester.  All
        requester operations are done using this object and its
        contents.
        """
        self.priority = priority
        """ Default for new requests' priorities if none specified. """

        self.feedback = feedback        # requester feedback
        self.pub_topic = topic
        self.sub_topic = common.feedback_topic(uuid, topic)
        rospy.loginfo('ROCON requester feedback topic: ' + self.sub_topic)
        self.sub = rospy.Subscriber(self.sub_topic,
                                    SchedulerRequests,
                                    self._feedback)
        self.pub = rospy.Publisher(self.pub_topic,
                                   SchedulerRequests,
                                   latch=True)
        self.time_delay = rospy.Duration(1.0 / frequency)
        self._set_timer()

    def _feedback(self, msg):
        """ Scheduler feedback message handler. """
        # Make a new RequestSet of the scheduler replies from this message
        new_rset = transitions.RequestSet(msg.requests,
                                          self.requester_id,
                                          replies=True)
        self.rset.merge(new_rset)
        prev_rset = copy.deepcopy(self.rset)

        # invoke user-defined callback function
        self.feedback(self.rset)

        # reply immediately if callback changed anything
        if self.rset != prev_rset:      # callback changed the rset?
            #print('requester callback changed requests:\n' + str(prev_rset)
            #      + '\nto\n' + str(self.rset))
            self.send_requests()

    def _heartbeat(self, event):
        """ Scheduler request heartbeat timer handler.

        Triggered after nothing has been sent to the scheduler within
        the previous time_delay duration.  Sends another copy of the
        current request set to the scheduler.

        """
        self.send_requests()

    def new_request(self, resources, priority=None, uuid=None):
        """ Add a new scheduler request.

        Call this method for each desired new request, then invoke
        :py:meth:`.send_requests` to notify the scheduler.

        :param resources: ROCON resources requested
        :type resources: list of scheduler_msgs/Resource

        :param priority: Scheduling priority of this request.  If
            ``None`` provided, use this requester's priority.
        :type priority: int

        :param uuid: UUID_ of this request. If ``None`` provided, a
            random UUID will be assigned.
        :type uuid: :class:`uuid.UUID` or ``None``

        :returns: UUID (:class:`uuid.UUID`) assigned.
        :raises: :exc:`.WrongRequestError` if request already exists.
        """
        if priority is None:
            priority = self.priority
        if uuid is None:
            uuid = unique_id.fromRandom()
        if uuid in self.rset:
            raise transitions.WrongRequestError('UUID already in use.')
        msg = Request(id=unique_id.toMsg(uuid),
                      priority=priority,
                      resources=resources,
                      status=Request.NEW)
        self.rset[uuid] = transitions.ResourceRequest(msg)
        return uuid

    def send_requests(self):
        """ Send all current requests to the scheduler.

        Use this method after calling :py:meth:`.new_request` one or
        more times.  It will send them to the scheduler immediately.
        Otherwise, they would not go out until the next heartbeat
        timer event.

        .. note::

           A recent heartbeat may already have sent some recent
           requests.  This method just ensures they are all sent
           without further delay.

        """
        #print(str(self.rset))
        self.pub.publish(self.rset.to_msg())
        #self._set_timer()       # reset heartbeat timer

    def _set_timer(self):
        """ Schedule heartbeat timer callback. """
        if not rospy.is_shutdown():
            #self.timer.shutdown()
            #self.timer = rospy.Timer(self.time_delay,
            #                         self._heartbeat,
            #                         oneshot=True)
            self.timer = rospy.Timer(self.time_delay, self._heartbeat)
