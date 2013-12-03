Overview
========

The `rocon_scheduler_requests`_ ROS_ package provides Python and C++
interfaces for managing scheduler requests within the `Robotics in
Concert`_ (ROCON) framework.

The scheduler tries to allocate ROCON resources for multiple
requesting services with good throughput and latency.  It is a
resource allocator, but due to contention it is also a planner.  We
call it a "scheduler" because in many ways it resembles the early
batch `operating system schedulers`_, which assigned non-preemptable
resources like printers and tape drives.

This package provides high-level message interfaces for scheduler
implementations and also for ROCON services making resource requests.

*There are initially only some experimental Python modules.*  
*Similar C++ interfaces will be provided following that proof of concept.*

Scheduler Topics
----------------

The ROCON scheduler runs as a ROS node on same master as the ROCON
conductor, the rocon services and other Solution components.  It
subscribes to an allocation topic named **/rocon_scheduler** of type
`scheduler_msgs/SchedulerRequests`_.  Any ROCON service or application
sending messages to that topic is called a **requester**.  Each
requester assigns itself a `universally unique identifier`_ and
subscribes to a feedback topic using the hexadecimal string
representation of its UUID, in the form
**/rocon_scheduler_0123456789abcdef0123456789abcdef**. The scheduler
will provide status feedback on that topic via
`scheduler_msgs/SchedulerRequests`_ messages.

Scheduler Resource Requests
---------------------------

Each `scheduler_msgs/SchedulerRequests`_ message describes all
resources currently desired by that requester.  The status each
resource request is passed back and forth between the requester and
the scheduler via `scheduler_msgs/Request`_ elements contained in the
allocation and feedback topics.

Handling these messages and managing the states of each resource
request can be quite tricky, because state changes flow over the two
topics simultaneously.  So, both schedulers and requesters need to
perform state transitions carefully and consistently for every
request.  

This package provides Python and C++ interface implementations for
schedulers and requesters to perform those transitions correctly.

.. _`operating system schedulers`: http://en.wikipedia.org/wiki/Scheduling_(computing)
.. _`Robotics in Concert`: http://www.robotconcert.org/wiki/Main_Page
.. _`rocon_scheduler_requests`: http://wiki.ros.org/rocon_scheduler_requests
.. _ROS: http://wiki.ros.org
.. _`scheduler_msgs/Request`: https://github.com/jack-oquin/rocon_msgs/blob/hydro-devel/scheduler_msgs/msg/Request.msg
.. _`scheduler_msgs/SchedulerRequests`: https://github.com/jack-oquin/rocon_msgs/blob/hydro-devel/scheduler_msgs/msg/SchedulerRequests.msg
.. _`universally unique identifier`: http://en.wikipedia.org/wiki/Universally_unique_identifier
