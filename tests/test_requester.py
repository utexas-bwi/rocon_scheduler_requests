#!/usr/bin/env python

# enable some python3 compatibility options:
from __future__ import absolute_import, print_function, unicode_literals

import sys
import unittest

import uuid                     # standard Python module

from uuid_msgs.msg import UniqueID
import unique_id

# module being tested:
import scheduler_request_manager.requester as requester

class TestSchedulerRequestManager(unittest.TestCase):
    """Unit tests for rocon scheduler request manager.
    """

    def test_empty_request(self):
        """ Issue empty allocation request.
        """
        r = requester.Requester()

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('scheduler_request_manager',
                    'test_scheduler_request_manager_py',
                    TestSchedulerRequestManager)
