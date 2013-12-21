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
.. module:: transitions

This module tracks resource request state transitions, which occur as
`scheduler_msgs/Request`_ messages flow between schedulers and
requesters.

.. warning::

   Some of the ``scheduler_msgs.Request.status`` labels have changed,
   and older scheduler_msgs_ releases use different names.  As an
   interim work-around, this module provides a compatible Request
   message which works with either message version.  Use it like
   this::

       from rocon_scheduler_requests.transitions import Request

   Once all the released scheduler_msgs_ labels have been updated,
   switch back to the usual message import::

       from scheduler_msgs.msg import Request

As individual ``Request`` messages are passed back and forth between
the original requester and the scheduler, their ``status`` passes
through several state transitions.  States created by the scheduler
are gray, and transitions initiated by the requester are dashed:

.. graphviz:: state_transitions.dot

.. _scheduler_msgs:
    http://wiki.ros.org/scheduler_msgs
.. _`scheduler_msgs/Request`:
    http://docs.ros.org/api/scheduler_msgs/html/msg/Request.html

"""

# (unicode_literals not compatible with python2 uuid module)
from __future__ import absolute_import, print_function

# Ros dependencies
import rospy
import unique_id
from scheduler_msgs.msg import SchedulerRequests

# temporarily provide status label backwards compatibility
from scheduler_msgs.msg import Request
if not hasattr(Request, 'CLOSED'):
    # Define revised Request.status labels.
    # See: (robotics-in-concert/rocon_msgs#60)
    Request.CANCELING = Request.RELEASING
    Request.CLOSED = Request.RELEASED

from . import TransitionError, WrongRequestError

## State transition merge table.
#
#  An immutable set of (old, new) status pairs.  All pairs in the
#  table are considered valid state transitions.  Any others are not.
#
#  This table is only used for merging scheduler and requester request
#  sets when a new message arrives.  Data from the new message are
#  ignored unless the corresponding transition appears here.
#
TRANS_TABLE = frozenset([
    (Request.CANCELING, Request.CANCELING),
    (Request.CANCELING, Request.CLOSED),

    (Request.GRANTED, Request.CANCELING),
    (Request.GRANTED, Request.GRANTED),
    (Request.GRANTED, Request.PREEMPTING),

    (Request.NEW, Request.CANCELING),
    (Request.NEW, Request.GRANTED),
    (Request.NEW, Request.PREEMPTING),
    (Request.NEW, Request.WAITING),

    (Request.PREEMPTING, Request.CANCELING),
    (Request.PREEMPTING, Request.CLOSED),
    (Request.PREEMPTING, Request.PREEMPTING),

    (Request.RESERVED, Request.CANCELING),
    (Request.RESERVED, Request.GRANTED),
    (Request.RESERVED, Request.PREEMPTING),
    (Request.RESERVED, Request.RESERVED),
    (Request.RESERVED, Request.WAITING),

    (Request.WAITING, Request.CANCELING),
    (Request.WAITING, Request.GRANTED),
    (Request.WAITING, Request.PREEMPTING),
    (Request.WAITING, Request.WAITING)])


class _EventTranitions:
    """
    Define valid status transitions for a given event type.

    :param name: Human-readable name for this event type.
    :type name: str
    :param trans: Dictionary of valid status transitions.
    :type trans: dict

    """
    def __init__(self, name, trans):
        self.name = name
        """ Name of this event type. """
        self.trans = trans
        """ Dictionary of valid status transitions. """

##  Requester transitions:
#
EVENT_CANCEL = _EventTranitions('cancel', {
    Request.CANCELING: Request.CANCELING,
    Request.CLOSED: Request.CLOSED,
    Request.GRANTED: Request.CANCELING,
    Request.NEW: Request.CANCELING,
    Request.PREEMPTING: Request.CANCELING,
    Request.RESERVED: Request.CANCELING,
    Request.WAITING: Request.CANCELING,
    })

##  Scheduler transitions:
#
EVENT_CLOSE = _EventTranitions('close', {
    Request.CANCELING: Request.CLOSED,
    Request.CLOSED: Request.CLOSED,
    Request.PREEMPTING: Request.CLOSED,
    })

EVENT_GRANT = _EventTranitions('grant', {
    Request.GRANTED: Request.GRANTED,
    Request.NEW: Request.GRANTED,
    Request.RESERVED: Request.GRANTED,
    Request.WAITING: Request.GRANTED,
    })

EVENT_PREEMPT = _EventTranitions('preempt', {
    Request.CANCELING: Request.CANCELING,
    Request.CLOSED: Request.CLOSED,
    Request.GRANTED: Request.PREEMPTING,
    Request.NEW: Request.NEW,
    Request.PREEMPTING: Request.PREEMPTING,
    Request.RESERVED: Request.RESERVED,
    Request.WAITING: Request.WAITING,
    })

EVENT_WAIT = _EventTranitions('wait', {
    Request.NEW: Request.WAITING,
    Request.RESERVED: Request.WAITING,
    Request.WAITING: Request.WAITING,
    })


class RequestBase:
    """
    Base class for tracking the status of a single resource request.

    *Not for general use.*

    :param msg: ROCON scheduler request message.
    :type msg: scheduler_msgs/Request

    Use one of these derived classes, depending on the direction of
    message flow:

    * :class:`.ResourceRequest` requester -> scheduler
    * :class:`.ResourceReply` scheduler -> requester

    .. describe:: str(rq)

       :returns: String representation of this resource request.

    """
    def __init__(self, msg):
        """ Constructor. """
        self.msg = msg
        """ Corresponding *scheduler_msgs/Request*. """

    def get_uuid(self):
        """ :returns: UUID of this request.
        :rtype: :class:`uuid.UUID`
        """
        return unique_id.fromMsg(self.msg.id)

    def __str__(self):
        """ Generate string representation. """
        return 'id: ' + str(unique_id.fromMsg(self.msg.id)) \
            + '\n    priority: ' + str(self.msg.priority) \
            + '\n    resources: ' + self._str_resources() \
            + '\n    status: ' + str(self.msg.status)

    def _str_resources(self):
        """ Format requested resource into a human-readable string. """
        retval = ''
        for res in self.msg.resources:
            retval += '\n      ' + res.platform_info + '/' + res.name
        return retval

    def _transition(self, event):
        """
        Update status for this resource request.

        :param event: Transition table for this type of *event*.
        :type event: :class:`._EventTranitions`

        :raises: :exc:`.TransitionError` if not a valid transition.

        """
        new_status = event.trans.get(self.msg.status)
        if new_status is None:
            raise TransitionError('invalid event ' + event.name
                                  + ' in state ' + str(self.msg.status))
        self.msg.status = new_status

    def _validate(self, new_status):
        """
        Validate status update for this resource request.

        :param new_status: Proposed new status for this request.

        :returns: ``True`` if this is a valid state transition.

        """
        return (self.msg.status, new_status) in TRANS_TABLE


class ResourceRequest(RequestBase):
    """
    This class represents a single resource request from the point of
    view of the original requester.

    :param msg: ROCON scheduler request message.
    :type msg: scheduler_msgs/Request

    Provides all attributes defined for :class:`.RequestBase`.

    """
    def cancel(self):
        """ Cancel a previously-requested resource.

        *Always valid for requesters.*

        """
        self._transition(EVENT_CANCEL)

    def _reconcile(self, update):
        """
        Merge scheduler updates with requester status.

        :param update: Latest information for this request, or
                       ``None`` if no longer present.
        :type update: :class:`.ResourceReply` or ``None``

        :raises: :exc:`.WrongRequestError`

        Only the requester creates new requests.  If something is
        missing from the scheduler feedback, that just means the
        scheduler has not yet heard about it.

        """
        if update is None:      # this request not yet known to scheduler?
            return              # leave it alone
        if update.get_uuid() != self.get_uuid():
            raise WrongRequestError('UUID does not match')
        if self._validate(update.msg.status):
            self.msg.status = update.msg.status
            self.msg.priority = update.msg.priority
            self.msg.resources = update.msg.resources
            if update.msg.availability != rospy.Time():
                self.msg.availability = update.msg.availability

    def release(self):
        ##   comment out docstring to remove from documentation:
        #""" Release a previously-requested resource.
        #
        #.. deprecated:: 0.0.1
        #   use :py:meth:`.cancel` instead.
        #
        #"""
        self.cancel()


class ResourceReply(RequestBase):
    """
    This class represents a single resource reply from the point of
    view of the scheduler.

    :param msg: ROCON scheduler request message.
    :type msg: scheduler_msgs/Request

    Provides all attributes defined for :class:`.RequestBase`.

    """
    def abort(self):
        ##   comment out docstring to remove from documentation:
        #""" Abort a request due to internal failure (always valid).
        #
        #.. deprecated:: 0.0.1
        #   use :py:meth:`.preempt` instead.
        #
        #"""
        self.preempt()

    def close(self):
        """ Close resource request.

        :raises: :exc:`.TransitionError`

        """
        self._transition(EVENT_CLOSE)

    def free(self):
        ##   comment out docstring to remove from documentation:
        #""" Free up previously-assigned resources that were canceled.
        #
        #:raises: :exc:`.TransitionError`
        #
        #.. deprecated:: 0.0.1
        #   use :py:meth:`.close` instead.
        #"""
        self.close()

    def grant(self, resources):
        """ Grant some specific requested resources.

        :param resources: Exact resources granted.
        :type resources: list of ``scheduler_msgs/Resource``
        :raises: :exc:`.TransitionError`

        The caller is responsible for ensuring that the granted
        resources really do fully satisfy this request.

        """
        self._transition(EVENT_GRANT)
        self.msg.resources = resources

    def _reconcile(self, update):
        """
        Merge updated request with current scheduler status.

        :param update: Latest information for this request, or
                       ``None`` if no longer present.
        :type update: :class:`.ResourceRequest` or ``None``

        :raises: :exc:`.WrongRequestError`

        """
        if update is None:      # this request not mentioned in updates
            update = ResourceRequest(self.msg)
            update.msg.status = Request.CLOSED
        elif update.get_uuid() != self.get_uuid():
            raise WrongRequestError('UUID does not match')
        if self._validate(update.msg.status):
            self.msg.status = update.msg.status
            self.msg.hold_time = update.msg.hold_time
            if (update.msg.status == Request.RESERVED
                    and update.msg.availability != rospy.Time()):
                self.msg.availability = update.msg.availability

    def preempt(self):
        """ Preempt a previously granted request.

        Always valid for the scheduler, but has no effect unless the
        request was previously granted.

        """
        self._transition(EVENT_PREEMPT)

    def reject(self):
        ##   comment out docstring to remove from documentation:
        #""" Reject an invalid request.
        #
        #:raises: :exc:`.TransitionError`
        #
        #.. deprecated:: 0.0.1
        #   use :py:meth:`.preempt` instead.
        #
        #"""
        self.preempt()

    def wait(self):
        """
        Put request in wait status until a suitable resource is available.

        :raises: :exc:`.TransitionError`
        """
        self._transition(EVENT_WAIT)


class RequestSet:
    """
    This class is a container for all the resource requests or
    responses for a single requester.  It acts like a dictionary.

    :param requests: List of ``Request`` messages, typically from the
        ``requests`` component of a ``SchedulerRequests`` message.
    :param requester_id: (:class:`uuid.UUID`) Unique ID this requester.
    :param contents: Class from which to instantiate set members.

    :class:`.RequestSet` supports these standard container operations:

    .. describe:: len(rset)

       :returns: The number of requests in the set.

    .. describe:: rset[uuid]

       :returns: The item corresponding to *uuid*.
       :raises: :exc:`KeyError` if no such request.

    .. describe:: rset[uuid] = msg

       Assign a Request message for this *uuid*.

       :param uuid: (:class:`uuid.UUID`) UUID of the request.
       :param msg: (*scheduler_msgs/Request*) message to add.

    .. describe:: rset == other

       :returns: ``True`` if *rset* and *other* have the same contents.

        Ignores the difference between request and reply messages.

    .. describe:: rset != other

       :returns: ``True`` if *rset* and *other* have different contents.

        Ignores the difference between request and reply messages.

    .. describe:: str(rset)

       :returns: String representation of :class:`.RequestSet`.

    .. describe:: uuid in rset

       :returns: ``True`` if *rset* has a key *uuid*, else ``False``.

    .. describe:: uuid not in rset

       Equivalent to ``not uuid in rset``.

    These attributes are also provided:

    """

    def __init__(self, requests, requester_id, contents=ResourceRequest):
        """ Constructor. """
        self.requester_id = requester_id
        """ :class:`uuid.UUID` of this requester. """
        self.contents = contents
        """ Type of objects this request set contains. """
        self.requests = {}
        for msg in requests:
            rq = self.contents(msg)
            self.requests[rq.get_uuid()] = rq

    def __contains__(self, uuid):
        """ Request set membership. """
        return uuid in self.requests

    def __eq__(self, other):
        """ RequestSet equality operator. """
        if self.requester_id != other.requester_id:
            return False        # different requester
        if set(self.requests.keys()) != set(other.requests.keys()):
            return False        # different request IDs
        for rqid, rq in self.requests.items():
            ## this does not work:
            #if rq.msg != other[rqid].msg:
            #    return False    # contents of some request changed
            other_msg = other[rqid].msg
            if rq.msg.status != other_msg.status:
                return False
            if rq.msg.priority != other_msg.priority:
                return False
            if rq.msg.availability != other_msg.availability:
                return False
            if rq.msg.hold_time != other_msg.hold_time:
                return False
            if rq.msg.resources != other_msg.resources:
                return False
        return True

    def __getitem__(self, uuid):
        """
        :param uuid: UUID of desired request.
        :type uuid: :class:`uuid.UUID`

        :returns: named item.
        :raises: :exc:`KeyError` if no such request
        """
        return self.requests[uuid]

    def __len__(self):
        """ Number of requests. """
        return len(self.requests)

    def __ne__(self, other):
        """ RequestSet != operator. """
        return not self == other

    def __setitem__(self, uuid, msg):
        """ Assign a Request message for this *uuid*. """
        self.requests[uuid] = self.contents(msg)

    def __str__(self):
        rval = 'requester_id: ' + str(self.requester_id) + '\nrequests:'
        for rq in self.requests.values():
            rval += '\n  ' + str(rq)
        return rval

    def get(self, uuid, default=None):
        """ Get request, if known.

        :param uuid: UUID of desired request.
        :type uuid: :class:`uuid.UUID`
        :param default: value to return if no such request.

        :returns: named item, if successful; else *default*.

        """
        return self.requests.get(uuid, default)

    def items(self):
        """
        :returns: all (key, value) pairs for this :class:`.RequestSet`.
        :rtype: list (Python2) or dictionary view (Python3)

        """
        return self.requests.items()

    def keys(self):
        """
        :returns: all UUIDs for this :class:`.RequestSet`.
        :rtype: list (Python2) or dictionary view (Python3)

        """
        return self.requests.keys()

    def _list_requests(self):
        """
        Return a list of resource requests suitable for inclusion in
        a ``SchedulerRequests`` message.

        :returns: list of *scheduler_msgs/Request* messages.

        """
        return [rq.msg for rq in self.requests.values()]

    def merge(self, updates):
        """
        Merge new request information into this RequestSet.

        :param updates: Request set containing updated information.
        :type updates: :class:`.RequestSet`

        This is *not* a :py:meth:`set.update` or :py:meth:`set.union`
        operation:

        * New elements from the *updates* will be added, but only if
          they are in an initial state (NEW or RESERVED).

        * Existing elements will be reconciled with the corresponding
          *updates* status.

        * Any element reaching a terminal status known by both sides
          of the protocol will be deleted.

        """
        # Add any new requests not previously known.
        for rid, new_rq in updates.items():
            if (rid not in self.requests and
                    (new_rq.msg.status == Request.NEW or
                     new_rq.msg.status == Request.RESERVED)):
                self.requests[rid] = self.contents(new_rq.msg)

        # Reconcile each existing request with the updates.  Make a
        # copy of the dictionary items, so it can be altered in the loop.
        for rid, rq in self.requests.items():
            new_rq = updates.get(rid)
            if ((rq.msg.status == Request.CANCELING and
                    new_rq.msg.status == Request.CLOSED)
                    or (rq.msg.status == Request.CLOSED and
                        new_rq is None)):
                del self.requests[rid]  # no longer needed
            else:
                rq._reconcile(new_rq)

    def to_msg(self, stamp=None):
        """ Convert to ROS ``scheduler_msgs/SchedulerRequest`` message.

        :param stamp: Time stamp for message header. If ``None``, use
            current time.
        :type stamp: rospy.Time

        :returns: corresponding ``scheduler_msgs/SchedulerRequests``

        """
        msg = SchedulerRequests(requester=unique_id.toMsg(self.requester_id),
                                requests=self._list_requests())
        if stamp is None:
            stamp = rospy.Time.now()
        msg.header.stamp = stamp
        return msg

    def values(self):
        """
        :returns: all requests for this :class:`.RequestSet`.
        :rtype: list (Python2) or dictionary view (Python3)

        """
        return self.requests.values()
