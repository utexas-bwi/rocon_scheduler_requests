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

.. _ROCON: http://www.robotconcert.org/wiki/Main_Page
.. _`Robotics in Concert`: http://www.robotconcert.org/wiki/Main_Page
.. _`scheduler_msgs/Resource`:
    http://docs.ros.org/api/scheduler_msgs/html/msg/Resource.html

"""

from __future__ import absolute_import, print_function, unicode_literals

import re
import uuid
import unique_id
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


def rocon_name(platform_info):
    """ Generate canonical ROCON resource name.

    :param platform_info: Platform info string from a
        :class:`.RoconResource`, ``scheduler_msgs/Resource`` message,
        or other resource representation.

    :returns: (str) Canonical ROCON name for this resource.

    A fully-resolved canonical name uniquely describes each resource
    within a ROCON_ Concert.  Some requests may match multiple
    resources by embedding regular expression syntax in the name.

    If the *platform_info* is not already a ROCON name starting with
    'rocon://', assume it may use the dotted syntax and convert bash
    wildcard asterisks to the equivalent Python regular expression.

    """
    if platform_info[0:8] == 'rocon://':
        return platform_info            # already canonical

    # Assume dotted representation, convert that to canonical format.
    retval = 'rocon://'
    for part in platform_info.split('.'):
        if part == '*':                 # bash wildcard syntax?
            part = '\\.*'               # convert to Python RE
        retval += '/' + part
    return retval


class RoconResource:
    """
    Base class for tracking the status of a single ROCON_ resource.

    :param msg: ROCON scheduler resource message.
    :type msg: scheduler_msgs/Resource

    .. describe:: hash(res)

       :returns: Hash key for this resource.

    .. describe:: res == other

       :returns: True if this :class:`.RoconResource` is equal to the *other*.

    .. describe:: res != other

       :returns: True if this :class:`.RoconResource` differs from the *other*.

    .. describe:: str(res)

       :returns: Human-readable string representation of this
           :class:`.RoconResource`.

    These attributes are also provided:

    """
    def __init__(self, msg):
        """ Constructor. """
        self.platform_info = rocon_name(msg.platform_info)
        """ Fully-resolved canonical ROCON resource name. """
        self.rapps = set([msg.name])
        """ The :class:`set` of ROCON app name strings this platform
        advertises. """
        self.owner = None
        """ :class:`uuid.UUID` of request to which this resource is
        currently assigned, or ``None``.
        """
        self.status = AVAILABLE
        """ Current status of this resource. """

    def __eq__(self, other):
        """ RoconResource equality operator. """
        if self.platform_info != other.platform_info:
            return False
        if self.rapps != other.rapps:
            return False                # different rapps advertised
        if self.owner != other.owner:
            return False
        if self.status != other.status:
            return False
        return True

    def __hash__(self):
        """ :returns: hash value for this resource. """
        return hash(rocon_name(self.platform_info))

    def __ne__(self, other):
        """ RoconResource != operator. """
        return not self == other

    def __str__(self):
        """ Format resource into a human-readable string. """
        rappstr = ''
        for rapp_name in self.rapps:
            rappstr += '\n    ' + str(rapp_name)
        return (rocon_name(self.platform_info)
                + ', status: ' + str(self.status)
                + '\n  owner: ' + str(self.owner)
                + '\n  rapps:' + rappstr)

    def allocate(self, request_id):
        """ Allocate this resource.

        :param request_id: New owner of this resource.
        :type request_id: :class:`uuid.UUID`

        :raises: :exc:`.ResourceNotAvailableError` if not available
        """
        if (self.status != AVAILABLE):
            raise ResourceNotAvailableError('resource not available: '
                                            + rocon_name(self.platform_info))
        assert self.owner is None
        self.owner = request_id
        self.status = ALLOCATED

    def match(self, pattern):
        """ Match this resource to a wildcard pattern.

        :param pattern: Name to match with possible wildcard request.
        :type pattern: ``scheduler_msgs/Resource``
        :returns: True if this specific resource matches.

        The rapp name in the *pattern* must be one of those advertised
        by this ROCON resource.  The *platform_info* in the *pattern*
        may include Python regular expression syntax for matching
        multiple resource names.

        TODO: If the *pattern* is not a ROCON name starting with
        'rocon://', assume it may use bash wildcard syntax and convert
        that to an equivalent Python regular expression.

        """
        if pattern.name not in self.rapps:
            return False                # rapp not advertised here
        return re.match(rocon_name(pattern.platform_info), self.platform_info)

    def release(self, request_id):
        """ Release this resource.

        :param request_id: Owning request.
        :type request_id: :class:`uuid.UUID`

        :raises: :exc:`.ResourceNotOwnedError` if not available
        """
        if (self.owner != request_id or request_id is None):
            raise ResourceNotOwnedError('resource not owned by '
                                        + str(request_id) + ': '
                                        + rocon_name(self.platform_info))
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

    .. describe:: key in resources

       :returns: ``True`` if *resources* contains *key*, else ``False``.

    .. describe:: key not in resources

       Equivalent to ``not key in resources``.

    .. describe:: len(resources)

       :returns: The number of resources in the set.

    .. describe:: resources[key]

       :param key: (str) A ROCON resource name.
       :returns: The :class:`RoconResource` corresponding to *key*.
       :raises: :exc:`KeyError` if no such *key*.

    .. describe:: resources[key] = res

       Assign a :class:`.RoconResource` to this *key*.

       :param key: (str) A ROCON resource name.
       :param res: Resource to add.
       :type res: :class:`.RoconResource` or ``scheduler_msgs/Resource``

    .. describe:: resources == another

       :returns: True if this :class:`.ResourceSet` is equal to *another*.

    .. describe:: resources != another

       :returns: True if this :class:`.ResourceSet` and *another* have
           different contents.

    .. describe:: str(resources)

       :returns: Human-readable string representation of this
           :class:`.ResourceSet`.

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
            if res != other[res_id]:
                return False
        return True

    def __getitem__(self, key):
        """
        :param key: Key of desired resource.

        :returns: named item.
        :raises: :exc:`KeyError` if no such request
        """
        return self.resources[hash(key)]

    def __len__(self):
        """ Number of resources. """
        return len(self.resources)

    def __ne__(self, other):
        """ ResourceSet != operator. """
        return not self == other

    def __setitem__(self, key, res):
        """ Assign a :class:`.RoconResource` to this *key*. """
        if not isinstance(res, RoconResource):
            res = RoconResource(res)    # make a RoconResource instance
        self.resources[hash(key)] = res

    def __str__(self):
        """ Format resource set into a human-readable string. """
        res_str = ''
        for res in self.resources.values():
            res_str += '\n  ' + str(res)
        return ('ROCON resource set:' + res_str)

    def get(self, key, default=None):
        """ Get resource, if known.

        :param key: ROCON name of desired resource.
        :type key: str
        :param default: value to return if no such resource.

        :returns: named :class:`.RoconResource` if successful, else *default*.

        """
        return self.resources.get(hash(key), default)
