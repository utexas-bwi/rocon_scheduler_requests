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
.. module:: resources

This module tracks resources and their allocation.  The ROS
`scheduler_msgs/Resource`_ message describes resources used by the
`Robotics in Concert`_ (ROCON) project.

.. _`Robotics in Concert`: http://www.robotconcert.org/wiki/Main_Page
.. _`scheduler_msgs/Resource`:
    http://docs.ros.org/api/scheduler_msgs/html/msg/Resource.html

"""

from __future__ import absolute_import, print_function, unicode_literals

import uuid
from scheduler_msgs.msg import Resource

## Resource states:
AVAILABLE = 0
ALLOCATED = 1
MISSING = 2


class ResourceNotAvailableError(Exception):
    """ Error exception: resource not available. """
    pass


class ResourceNotOwnedError(Exception):
    """ Error exception: resource not owned. """
    pass


class RoconResource:
    """
    Base class for tracking the status of a single ROCON resource.

    :param msg: ROCON scheduler resource message.
    :type msg: scheduler_msgs/Resource

    :param request_id: ROCON scheduler identifier of request to which
        this resource is assigned.
    :type request_id: :class:`uuid.UUID`

    .. describe:: hash(res)

       :returns: Hash key for this resource request.

    .. describe:: str(res)

       :returns: String representation of this resource request.

    These attributes are also provided:

    """
    def __init__(self, msg, status=AVAILABLE, request_id=None):
        """ Constructor. """
        self.msg = msg
        """ Corresponding *scheduler_msgs/Resource*. """
        self.owner = request_id
        """ :class:`uuid.UUID` of request to which this resource is
        currently assigned, or ``None``.
        """
        self.status = status
        """ Current status of this resource. """

    def __hash__(self):
        """ :returns: hash value for this resource. """
        return hash(str(self))

    def __str__(self):
        """ Format resource into a human-readable string. """
        return self.msg.platform_info + '/' + self.msg.name

    def allocate(self, request_id):
        """ Allocate this resource.

        :param request_id: New owner of this resource.
        :type request_id: :class:`uuid.UUID`

        :raises: :exc:`.ResourceNotAvailableError` if not available
        """
        if (self.status != AVAILABLE):
            raise ResourceNotAvailableError('resource not available: '
                                            + str(self))
        assert self.owner is None
        self.owner = request_id
        self.status = ALLOCATED

    def release(self, request_id):
        """ Release this resource.

        :param request_id: Owning request.
        :type request_id: :class:`uuid.UUID`

        :raises: :exc:`.ResourceNotOwnedError` if not available
        """
        if (self.owner != request_id or request_id is None):
            raise ResourceNotOwnedError('resource not owned by '
                                        + str(request_id) + ': ' + str(self))
        self.owner = None
        if self.status == ALLOCATED:    # not gone missing?
            self.status = AVAILABLE


class ResourceSet:
    """
    This class is a container for :class:`.RoconResource` objects
    known to the scheduler.  It acts like a dictionary.

    :param resource_list: An optional list of ``Resource`` messages,
        like the ``resources`` component of a ``scheduler_msgs/Request``
        message.

    :class:`.ResourceSet` supports these standard container operations:

    .. describe:: len(resources)

       :returns: The number of resources in the set.

    .. describe:: resources[key]

       :returns: The item corresponding to *key*.
       :raises: :exc:`KeyError` if no such *key*.

    .. describe:: resources[key] = resource

       Assign a :class:`.RoconResource` for this *key*.

       :param key: (str) ROCON name of the resource.
       :param resource: (``scheduler_msgs/Resource``) message to add.

    .. describe:: str(resources)

       :returns: String representation of :class:`.ResourceSet`.

    .. describe:: res in resources

       :returns: ``True`` if *resources* has a key *res*, else ``False``.

    .. describe:: res not in resources

       Equivalent to ``not res in resources``.

    These attributes are also provided:

    """
    def __init__(self, resource_list=[]):
        """ Constructor. """
        self.resources = {}
        """ Dictionary of known :class:`.RoconResource` objects. """
        for res in resource_list:
            rocon_res = RoconResource(res)
            self.resources[hash(rocon_res)] = rocon_res

    def __contains__(self, res):
        """ Resource set membership. """
        return hash(res) in self.resources

    def __eq__(self, other):
        """ ResourceSet equality operator. """
        if set(self.resources.keys()) != set(other.resources.keys()):
            return False        # different resources hash IDs
        for res_id, res in self.resources.items():
            ## this does not work:
            if res.msg != other[res_id].msg:
                return False    # contents of some request changed
        return True

    def __getitem__(self, key):
        """
        :param key: Key of desired resource.

        :returns: named item.
        :raises: :exc:`KeyError` if no such request
        """
        return self.resources[key]

    def __len__(self):
        """ Number of resources. """
        return len(self.resources)

    def __ne__(self, other):
        """ ResourceSet != operator. """
        return not self == other
