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
import sets

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


def to_Request(resource, uuid=None):
    """ Create a new rocon scheduler request ROS message.

    :param resource: Rocon resource requested, may contain wild cards.
    :type resource: rocon_std_msgs/PlatformInfo
    :param uuid: UUID_ of this request. If ``None`` provided, a random
                 uuid will be assigned.
    :type uuid: :class:`uuid.UUID` or ``None``

    :returns: a new scheduler request ROS message
    :rtype: scheduler_msgs/Request

    """
    if uuid is None:
        uuid = unique_id.fromRandom()
    return Request(id=unique_id.toMsg(uuid),
                   resource=resource,
                   status=Request.NEW)


# State transition table.
#
# An immutable set of (old, new) status pairs.  All pairs in the table
# are considered valid state transitions.  Any others are not.
#
TRANS_TABLE = sets.ImmutableSet([(Request.NEW, Request.WAITING),
                                 (Request.NEW, Request.GRANTED),
                                 (Request.NEW, Request.PREEMPTING),
                                 (Request.NEW, Request.ABORTED),
                                 (Request.WAITING, Request.GRANTED),
                                 (Request.WAITING, Request.PREEMPTING),
                                 (Request.WAITING, Request.RELEASING),
                                 (Request.WAITING, Request.ABORTED),
                                 (Request.GRANTED, Request.PREEMPTING),
                                 (Request.GRANTED, Request.RELEASING),
                                 (Request.GRANTED, Request.ABORTED),
                                 (Request.PREEMPTING, Request.PREEMPTED),
                                 (Request.PREEMPTING, Request.RELEASING),
                                 (Request.PREEMPTING, Request.ABORTED),
                                 (Request.PREEMPTED, Request.NEW),
                                 (Request.PREEMPTED, Request.ABORTED),
                                 (Request.RELEASING, Request.RELEASED),
                                 (Request.RELEASING, Request.ABORTED)])


class ResourceRequest:
    """
    This class tracks the status of a single resource request.

    :param msg: Rocon scheduler request message.
    :type msg: scheduler_msgs/Request

    """
    def __init__(self, msg):
        """ Constructor. """
        self.msg = msg
        """ Current ``scheduler_msgs/Request`` for this request. """

    def free(self):
        """ Free up previously-assigned resource that was released.

        :raises: :exc:`.TransitionError`
        """
        if not self.validate(Request.RELEASED):
            raise TransitionError('invalid resource release, status = '
                                  + str(self.msg.status))
        self.msg.status = Request.RELEASED

    def get_uuid(self):
        """ :returns: UUID of this request.
        :rtype: :class:`uuid.UUID`
        """
        return unique_id.fromMsg(self.msg.id)

    def grant(self, resource):
        """ Grant a specific requested resource.

        :param resource: Exact resource granted.
        :type resource: rocon_std_msgs/PlatformInfo
        :raises: :exc:`.TransitionError`
        :raises: :exc:`.ResourceNotRequestedError`

        """
        if not self.validate(Request.GRANTED):
            raise TransitionError('invalid resource grant, status = '
                                  + str(self.msg.status))
        if not self.matches(resource):
            raise ResourceNotRequestedError(str(resource)
                                            + ' does not match '
                                            + str(self.msg.resource))
        self.msg.status = Request.GRANTED
        self.msg.resource = resource

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

    def reconcile(self, update):
        """
        Merge updated status with this :class:`.ResourceRequest`.

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

    def release(self):
        """ Release a previously granted resource.

        :raises: :exc:`.TransitionError`

        """
        if not self.validate(Request.RELEASING):
            raise TransitionError('invalid resource release, status = '
                                  + str(self.msg.status))
        self.msg.status = Request.RELEASING

    def validate(self, new_status):
        """
        Validate status update for this :class:`.ResourceRequest`.

        :param new_status: Latest status provided for this request.
        :type new_status: :class:`.RequestSet`

        :returns ``True`` if update represents a valid state transition.

        """
        return (self.msg.status, new_status) in TRANS_TABLE

    def wait(self):
        """
        Put request in wait status until a suitable resource is available.

        :raises: :exc:`.TransitionError`
        """
        if not self.validate(Request.WAITING):
            raise TransitionError('invalid resource transition, status = '
                                  + str(self.msg.status))
        self.msg.status = Request.WAITING


class RequestSet:
    """
    This class is a container for all the resource requests from a
    single requester or message.  It acts like a dictionary.

    :param requests: list of ``Request`` messages, typically from the
                     ``requests`` component of an ``AllocateResources``
                     or ``SchedulerFeedback`` message.

    :class:`.RequestSet` supports these standard container operations:

    .. describe:: len(rset)

       :returns: the number of requesters in the set.

    .. describe:: rset[uuid]

       :returns: the resource request corresponding to *uuid*.
       :raises: :exc:`KeyError` if no such request.

    .. describe:: rset[uuid] = rq

       Define a new :class:`.ResourceRequest` for this UUID.

       :param uuid: UUID_ of the request.
       :type uuid: :class:`uuid.UUID`
       :param rq: request.
       :type rq: :class:`.ResourceRequest`

    .. describe:: uuid in rset

       :returns: ``True`` if *rset* has a key *uuid*, else ``False``.

    .. describe:: uuid not in rset

       Equivalent to ``not uuid in rset``.

    These methods are also provided:

    """

    def __init__(self, requests):
        """ Constructor. """
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

        :returns: named :class:`.ResourceRequest`.
        :raises: :exc:`KeyError` if no such request
        """
        return self.requests[uuid]

    #def __iter__(self):
    #    """ Resource Requests iterator. """
    #    self.iter_index = 0
    #    return self

    def __len__(self):
        """ Number of requests. """
        return len(self.requests)

    def __setitem__(self, uuid, rq):
        """ Add a resource request to the set. """
        self.requests[uuid] = rq

    def get(self, uuid, default=None):
        """ Get request, if known.

        :param uuid: UUID_ of desired request.
        :type uuid: :class:`uuid.UUID`
        :param default: value to return if no such request.

        :returns: named :class:`.ResourceRequest`, if successful;
                  otherwise default.

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
        an ``AllocateResources`` or ``SchedulerFeedback`` message.

        :returns: list of ``scheduler_msgs/Request`` messages.

        """
        return [rq.msg for rq in self.requests.itervalues()]

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

    #def next(self):
    #    """
    #    :returns: next request of iteration.
    #    :rtype: :class:`.ResourceRequest`
    #    :raises: :exc:`StopIteration` when finished.
    #    """

    def values(self):
        """
        :returns: all requests for this :class:`.RequestSet`.
        :rtype: list (Python2) or dictionary view (Python3)

        """
        return self.requests.values()
