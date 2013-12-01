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

This module tracks resource request state transtions as messages flow
between schedulers and requesters.

.. _UUID: http://en.wikipedia.org/wiki/Uuid

"""

# (unicode_literals not compatible with python2 uuid module)
from __future__ import absolute_import, print_function

import copy

# Ros dependencies
import rospy
from rocon_std_msgs.msg import PlatformInfo
from scheduler_msgs.msg import Request
import unique_id


class ResourceNotRequestedError(Exception):
    """ Error exception: resource does not match the request. """
    pass


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


class _RequestBase:
    """
    Base class for tracking the status of a single resource request.

    :param msg: Rocon scheduler request message.
    :type msg: scheduler_msgs/Request

    .. describe:: str(rq)

       :returns: String representation of this resource request.

    """
    def __init__(self, msg):
        """ Constructor. """
        self.msg = msg
        """ Current ``scheduler_msgs/Request`` for this request. """

    def __str__(self):
        """ :todo: add availability """
        return 'id: ' + str(unique_id.fromMsg(self.msg.id)) \
            + '\n    resource: ' + self.str_resource() \
            + '\n    status: ' + str(self.msg.status)

    def get_uuid(self):
        """ :returns: UUID of this request.
        :rtype: :class:`uuid.UUID`
        """
        return unique_id.fromMsg(self.msg.id)

    def matches(self, resource):
        """ Check whether a specific resource matches this request.

        :param resource: Exact resource to match.
        :type resource: rocon_std_msgs/PlatformInfo
        :returns: ``True`` if this resource matches.

        """
        if resource.os != self.msg.resource.os and \
                self.msg.resource.os != PlatformInfo.OS_ANY:
            return False
        if resource.version != self.msg.resource.version and \
                self.msg.resource.version != PlatformInfo.VERSION_ANY:
            return False
        if resource.system != self.msg.resource.system and \
                self.msg.resource.system != PlatformInfo.SYSTEM_ANY:
            return False
        if resource.platform != self.msg.resource.platform and \
                self.msg.resource.platform != PlatformInfo.PLATFORM_ANY:
            return False
        if resource.name != self.msg.resource.name and \
                self.msg.resource.name != PlatformInfo.NAME_ANY:
            return False
        return True

    def str_resource(self):
        """ Format requested resource into a human-readable string. """
        return self.msg.resource.os + '.' \
            + self.msg.resource.version + '.' \
            + self.msg.resource.system + '.' \
            + self.msg.resource.platform + '.' \
            + self.msg.resource.name

    def update_status(self, new_status):
        """
        Update status for this resource request.

        :param new_status: Desired status.

        :raises: :exc:`.TransitionError` if not a valid transition.

        """
        if not self.validate(new_status):
            raise TransitionError('invalid status transition from '
                                  + str(self.msg.status)
                                  + ' to ' + str(new_status))
        self.msg.status = new_status

    def validate(self, new_status):
        """
        Validate status update for this resource request.

        :param new_status: Proposed new status for this request.

        :returns: ``True`` if this is a valid state transition.

        """
        return (self.msg.status, new_status) in TRANS_TABLE


class ResourceRequest(_RequestBase):
    """
    This class represents a single request from this requester.

    :param msg: Rocon scheduler request message.
    :type msg: scheduler_msgs/Request

    Attributes:

    .. describe:: msg

       Current ``scheduler_msgs/Request`` for this request.

    .. describe:: str(rq)

       :returns: String representation of this resource request.

    """
    def abort(self):
        """ Abort a request due to internal failure (always valid). """
        self.update_status(Request.ABORTED)

    def free(self):
        """ Free up a previously-assigned resource that was released.

        :raises: :exc:`.TransitionError`
        """
        self.update_status(Request.RELEASED)

    def grant(self, resource):
        """ Grant a specific requested resource.

        :param resource: Exact resource granted.
        :type resource: rocon_std_msgs/PlatformInfo
        :raises: :exc:`.TransitionError`
        :raises: :exc:`.ResourceNotRequestedError`

        """
        self.update_status(Request.GRANTED)
        if not self.matches(resource):
            raise ResourceNotRequestedError(str(resource)
                                            + ' does not match '
                                            + str(self.msg.resource))
        self.msg.resource = resource

    def reconcile(self, update):
        """
        Merge updated status with this resource request.

        :param update: Latest information for this request, or
                       ``None`` if no longer present.
        :type update: :class:`.ResourceRequest` or ``None``

        :raises: :exc:`.WrongRequestError`

        """
        if update is None:      # this request not mentioned in updates
            update = copy.deepcopy(self)
            update.msg.status = Request.RELEASED
        elif update.get_uuid() != self.get_uuid():
            raise WrongRequestError('UUID does not match')
        if self.validate(update.msg.status):
            self.msg.status = update.msg.status
            self.msg.resource = update.msg.resource
            if update.msg.availability != rospy.Time():
                self.msg.availability = update.msg.availability

    def reject(self):
        """ Reject an invalid request.

        :raises: :exc:`.TransitionError`

        """
        self.update_status(Request.REJECTED)

    def release(self):
        """ Release a previously granted resource.

        :raises: :exc:`.TransitionError`

        """
        self.update_status(Request.RELEASING)

    def wait(self):
        """
        Put request in wait status until a suitable resource is available.

        :raises: :exc:`.TransitionError`
        """
        self.update_status(Request.WAITING)


class RequestSet:
    """
    This class is a container for all the resource requests or replies
    for a single requester.  It acts like a dictionary.

    :param requests: list of ``Request`` messages, typically from the
        ``requests`` component of a ``SchedulerRequests`` message.
    :param requester_id: (:class:`uuid.UUID`) Unique ID this requester.
    :param priority: Scheduling priority of this requester.

    :class:`.RequestSet` supports these standard container operations:

    .. describe:: len(rset)

       :returns: the number of requesters in the set.

    .. describe:: rset[uuid]

       :returns: the item corresponding to *uuid*.
       :raises: :exc:`KeyError` if no such request.

    .. describe:: rset[uuid] = rq

       Define a new item for this UUID.

       :param uuid: UUID_ of the request.
       :type uuid: :class:`uuid.UUID`
       :param rq: request.

    .. describe:: str(rset)

       :returns: String representation of :class:`RequestSet`.

    .. describe:: uuid in rset

       :returns: ``True`` if *rset* has a key *uuid*, else ``False``.

    .. describe:: uuid not in rset

       Equivalent to ``not uuid in rset``.

    These methods are also provided:

    """

    def __init__(self, requests, requester_id, priority=0):
        """ Constructor. """
        self.requester_id = requester_id
        """ :class:`uuid.UUID` of this requester. """
        self.priority = priority
        """ Current requester priority. """
        self.requests = {}
        for msg in requests:
            rq = ResourceRequest(msg)
            self.requests[rq.get_uuid()] = rq

    def __contains__(self, uuid):
        """ Request set membership. """
        return uuid in self.requests

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

    def __setitem__(self, uuid, rq):
        """ Add a resource request to the set. """
        self.requests[uuid] = rq

    def __str__(self):
        rval = 'requester_id: ' + str(self.requester_id) \
            + '\npriority: ' + str(self.priority) \
            + '\nrequests:'
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

    def list_requests(self):
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

        :todo: Pay attention to timing as messages and updates
               interleave.

        """
        # Reconcile each existing request with the updates.  Make a
        # copy of the dictionary items, so it can be altered in the loop.
        for rid, rq in self.requests.items():
            new_rq = updates.get(rid)
            if new_rq is None and rq.msg.status == Request.RELEASED:
                del self.requests[rid]  # no longer needed
            else:
                rq.reconcile(new_rq)

        # Add any new requests not previously known.
        for rid, new_rq in updates.items():
            if rid not in self.requests:
                self.requests[rid] = new_rq

    def values(self):
        """
        :returns: all requests for this :class:`.RequestSet`.
        :rtype: list (Python2) or dictionary view (Python3)

        """
        return self.requests.values()