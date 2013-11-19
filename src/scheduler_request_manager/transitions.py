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

"""

# (unicode_literals not compatible with python2 uuid module)
from __future__ import absolute_import, print_function

# Ros dependencies
from rocon_std_msgs.msg import PlatformInfo
from scheduler_msgs.msg import Request
import unique_id

class ResourceNotRequestedError(Exception):
    """ Error exception: resource does not match the request. """
    pass

class TransitionError(Exception):
    """ Error exception: invalid state transition. """
    pass

class ResourceRequest:
    """
    This class tracks the status of a single resource request.

    :param resource: Rocon resource requested, may contain wild cards.
    :type resource: rocon_std_msgs/PlatformInfo
    :param uuid: UUID_ of this request. If None provided, a random
                 uuid will be assigned.
    :type uuid: Standard Python :class:`uuid.UUID` object.

    :todo: implement a proper state transition function

    """

    def __init__(self, resource, uuid=None):
        """ Constructor. """ 
        if uuid is None:
            uuid = unique_id.fromRandom()
        self.msg = Request(id=unique_id.toMsg(uuid), resource=resource)

    def free(self):
        """ Free up previously-assigned resource that was released. 

        :raises: :class:`TransitionError`
        """
        if self.msg.status != Request.RELEASING:
            raise TransitionError('invalid resource release, status = '
                                  + str(self.msg.status))
        self.msg.status = Request.RELEASED

    def get_resource(self):
        """
        :returns: resource requested.
        :rtype: rocon_std_msgs/PlatformInfo

        """
        return self.msg.resource

    def get_status(self):
        """ :returns: current status of this request. """
        return self.msg.status

    def get_uuid(self):
        """ :returns: UUID of this request.
        :rtype: Standard Python :class:`uuid.UUID` object.
        """
        return unique_id.fromMsg(self.msg.id)

    def grant(self, resource):
        """ Grant a specific requested resource.

        :param resource: Exact resource granted.
        :type resource: rocon_std_msgs/PlatformInfo
        :raises: :class:`TransitionError`
        :raises: :class:`ResourceNotRequestedError`

        """
        if (self.msg.status != Request.NEW and
            self.msg.status != Request.WAITING):
            raise TransitionError('invalid resource grant, status = '
                                  + str(self.msg.status))
        self.msg.status = Request.GRANTED
        if not self.matches(resource):
            raise ResourceNotRequestedError(str(resource)
                                            + ' does not match '
                                            + str(self.msg.resource))
        self.msg.resource = resource

    def matches(self, resource):
        """ Check whether a specific resource matches this request.

        :param resource: Exact resource to match.
        :type resource: rocon_std_msgs/PlatformInfo
        :returns: true if this resource matches.

        """
        if (resource.os != self.msg.resource.os and
            self.msg.resource.os != PlatformInfo.OS_ANY):
            return False
        if (resource.version != self.msg.resource.version and
            self.msg.resource.version != PlatformInfo.VERSION_ANY):
            return False
        if (resource.system != self.msg.resource.system and
            self.msg.resource.system != PlatformInfo.SYSTEM_ANY):
            return False
        if (resource.platform != self.msg.resource.platform and
            self.msg.resource.platform != PlatformInfo.PLATFORM_ANY):
            return False
        if (resource.name != self.msg.resource.name and
            self.msg.resource.name != PlatformInfo.NAME_ANY):
            return False
        return True

    def release(self):
        """ Release a requested and currently granted resource. 

        :raises: :class:`TransitionError`

        """
        if self.msg.status != Request.GRANTED:
            raise TransitionError('invalid resource release, status = '
                                  + str(self.msg.status))
        self.msg.status = Request.RELEASING

    def update(self, msg):
        """ Update status based on message contents.

        :param msg: Latest message received.
        :type msg: scheduler_msgs/Request

        """
        pass                    # scaffolding
