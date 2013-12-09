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

This module tracks resource request state transitions as messages flow
between schedulers and requesters.

.. _UUID: http://en.wikipedia.org/wiki/Uuid

"""

# (unicode_literals not compatible with python2 uuid module)
from __future__ import absolute_import, print_function

# Ros dependencies
import rospy
from scheduler_msgs.msg import Request
from scheduler_msgs.msg import SchedulerRequests
import unique_id


class TransitionError(Exception):
    """ Error exception: invalid state transition. """
    pass


class WrongRequestError(Exception):
    """ Error exception: request update for the wrong UUID. """
    pass


# State transition table.
#
# An immutable set of (old, new) status pairs.  All pairs in the table
# are considered valid state transitions.  Any others are not.
#
TRANS_TABLE = frozenset([(Request.NEW, Request.ABORTED),
                         (Request.NEW, Request.GRANTED),
                         (Request.NEW, Request.PREEMPTING),
                         (Request.NEW, Request.REJECTED),
                         (Request.NEW, Request.WAITING),

                         (Request.WAITING, Request.ABORTED),
                         (Request.WAITING, Request.GRANTED),
                         (Request.WAITING, Request.PREEMPTING),
                         (Request.WAITING, Request.REJECTED),
                         (Request.WAITING, Request.RELEASING),

                         (Request.GRANTED, Request.ABORTED),
                         (Request.GRANTED, Request.PREEMPTING),
                         (Request.GRANTED, Request.RELEASING),

                         (Request.PREEMPTING, Request.ABORTED),
                         (Request.PREEMPTING, Request.PREEMPTED),
                         (Request.PREEMPTING, Request.REJECTED),
                         (Request.PREEMPTING, Request.RELEASING),

                         (Request.PREEMPTED, Request.ABORTED),
                         (Request.PREEMPTED, Request.NEW),
                         (Request.PREEMPTED, Request.REJECTED),

                         (Request.RELEASING, Request.ABORTED),
                         (Request.RELEASING, Request.REJECTED),
                         (Request.RELEASING, Request.RELEASED)])


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
        """ Corresponding ``scheduler_msgs/Request``. """

    def __str__(self):
        """ Generate string representation. """
        return 'id: ' + str(unique_id.fromMsg(self.msg.id)) \
            + '\n    priority: ' + str(self.msg.priority) \
            + '\n    resources: ' + self._str_resources() \
            + '\n    status: ' + str(self.msg.status)

    def get_uuid(self):
        """ :returns: UUID of this request.
        :rtype: :class:`uuid.UUID`
        """
        return unique_id.fromMsg(self.msg.id)

    def _str_resources(self):
        """ Format requested resource into a human-readable string. """
        retval = ''
        for res in self.msg.resources:
            retval += '\n      ' + res.platform_info + '/' + res.name
        return retval

    def _update_status(self, new_status):
        """
        Update status for this resource request.

        :param new_status: Desired status.

        :raises: :exc:`.TransitionError` if not a valid transition.

        """
        if not self._validate(new_status):
            raise TransitionError('invalid status transition from '
                                  + str(self.msg.status)
                                  + ' to ' + str(new_status))
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
    This class represents a single resource request flowing from
    requester to scheduler.

    :param msg: ROCON scheduler request message.
    :type msg: scheduler_msgs/Request

    Provides all attributes defined for :class:`.RequestBase`.

    """
    def _reconcile(self, update):
        """
        Merge scheduler updates with this request.

        :param update: Latest information for this request, or
                       ``None`` if no longer present.
        :type update: :class:`.ResourceReply` or ``None``

        :raises: :exc:`.WrongRequestError`

        """
        if update is None:      # this request not mentioned in updates
            update = ResourceReply(self.msg)
            update.msg.status = Request.RELEASED
        elif update.get_uuid() != self.get_uuid():
            raise WrongRequestError('UUID does not match')
        if self._validate(update.msg.status):
            self.msg.status = update.msg.status
            self.msg.priority = update.msg.priority
            self.msg.resources = update.msg.resources
            self.msg.availability = update.msg.availability
            if update.msg.availability != rospy.Time():
                self.msg.availability = update.msg.availability

    def release(self):
        """ Release a previously requested resource.

        :raises: :exc:`.TransitionError`

        """
        self._update_status(Request.RELEASING)


class ResourceReply(RequestBase):
    """
    This class represents a single resource reply flowing from
    scheduler to requester.

    :param msg: ROCON scheduler request message.
    :type msg: scheduler_msgs/Request

    Provides all attributes defined for :class:`.RequestBase`.

    """
    def abort(self):
        """ Abort a request due to internal failure (always valid). """
        self._update_status(Request.ABORTED)

    def free(self):
        """ Free up previously-assigned resources that were released.

        :raises: :exc:`.TransitionError`
        """
        self._update_status(Request.RELEASED)

    def grant(self, resources):
        """ Grant some specific requested resources.

        :param resources: Exact resources granted.
        :type resources: list of ``scheduler_msgs/Resource``
        :raises: :exc:`.TransitionError`

        The caller is responsible for ensuring that the granted
        resources really do fully satisfy this request.

        """
        self._update_status(Request.GRANTED)
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
            update.msg.status = Request.RELEASED
        elif update.get_uuid() != self.get_uuid():
            raise WrongRequestError('UUID does not match')
        if self._validate(update.msg.status):
            self.msg.status = update.msg.status
            self.msg.resources = update.msg.resources
            if update.msg.availability != rospy.Time():
                self.msg.availability = update.msg.availability

    def reject(self):
        """ Reject an invalid request.

        :raises: :exc:`.TransitionError`

        """
        self._update_status(Request.REJECTED)

    def wait(self):
        """
        Put request in wait status until a suitable resource is available.

        :raises: :exc:`.TransitionError`
        """
        self._update_status(Request.WAITING)


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

       :returns: the number of requesters in the set.

    .. describe:: rset[uuid]

       :returns: the item corresponding to *uuid*.
       :raises: :exc:`KeyError` if no such request.

    .. describe:: rset[uuid] = rq

       Define a new item for this UUID.

       :param uuid: (:class:`uuid.UUID`) UUID_ of the request.

       :param rq: (:class:`.ResourceRequest`) request or
           (:class:`.ResourceReply`) reply.

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
        :param uuid: UUID_ of desired request.
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

    def __setitem__(self, uuid, rq):
        """ Add a resource request to the set. """
        self.requests[uuid] = rq

    def __str__(self):
        rval = 'requester_id: ' + str(self.requester_id) + '\nrequests:'
        for rq in self.requests.values():
            rval += '\n  ' + str(rq)
        return rval

    def get(self, uuid, default=None):
        """ Get request, if known.

        :param uuid: UUID_ of desired request.
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

        :returns: list of ``scheduler_msgs/Request`` messages.

        """
        return [rq.msg for rq in self.requests.values()]

    def merge(self, updates):
        """
        Merge new request information into this RequestSet.

        :param updates: Request set containing updated information.
        :type updates: :class:`.RequestSet`

        This is *not* a :py:meth:`set.update` or :py:meth:`set.union`
        operation:

        * New elements from the *updates* will be added.

        * Existing elements will be reconciled with the corresponding
          *updates* status.

        * Any element reaching a terminal status known by both sides
          of the protocol will be deleted.

        """
        # Add any new requests not previously known.
        for rid, new_rq in updates.items():
            if rid not in self.requests:
                self.requests[rid] = self.contents(new_rq.msg)

        # Reconcile each existing request with the updates.  Make a
        # copy of the dictionary items, so it can be altered in the loop.
        for rid, rq in self.requests.items():
            new_rq = updates.get(rid)
            if ((rq.msg.status == Request.RELEASING and
                    new_rq.msg.status == Request.RELEASED)
                    or (rq.msg.status == Request.RELEASED and
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
