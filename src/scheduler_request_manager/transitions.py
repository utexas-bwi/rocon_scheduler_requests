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

# Ros messages
from scheduler_msgs.msg import Request

class TransitionError(Exception):
    """ Invalid state transition request error. """
    pass

class ResourceRequest:
    """
    This class tracks the status of a single resource request.
    """

    def __init__(self):
        """ Constructor. """
        self.msg = Request()

    def get_resource(self):
        """ Rocon resource accessor.

        :returns: resource requested.
        :rtype: rocon_std_msgs/PlatformInfo

        """
        return self.msg.resource

    def get_status(self):
        """ :returns: current status of this request. """
        return self.msg.status

    def grant(self, resource):
        """ Grant the requested resource.

        :param resource: Exact resource granted.
        :type resource: rocon_std_msgs/PlatformInfo
        :raises: TransitionError
        """
        # :todo: implement a proper state transition function
        if (self.msg.status != Request.NEW and
            self.msg.status != Request.WAITING):
            raise TransitionError('invalid resource grant, status = '
                                  + str(self.msg.status))
            
        self.msg.status = Request.GRANTED
        self.msg.resource = resource

    def release(self):
        """ Release a requested and currently granted resource. 

        :raises: TransitionError
        """
        # :todo: implement a proper state transition function
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
