Overview
========

The scheduler_request_manager is a ROS package providing Python and
C++ interfaces for managing rocon scheduler requests.

*There is initially only an experimental Python module.*  *A similar
C++ interface will be provided following that proof of concept.*

Scheduler Topics
----------------

The rocon scheduler runs as a ROS node on same master as the rocon
conductor, the rocon services and other Solution components.  It
subscribes to an allocation topic named **/rocon_scheduler** of type
`scheduler_msgs/AllocateResources`_.  Any rocon service or application
sending messages to that topic is called a **requester**.  Each
requester assigns itself a `Universally Unique Identifier`_ and
subscribes to a feedback topic using the string representation of its
unique ID, in the form
**/rocon_scheduler_01234567-89ab-cdef-0123-456789abcdef**. The
scheduler will provide status feedback on that topic via
`scheduler_msgs/SchedulerFeedback`_ messages.

Scheduler Resource Requests
---------------------------

Each `scheduler_msgs/AllocateResources`_ message describes all
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

.. _`scheduler_msgs/AllocateResources`: https://github.com/robotics-in-concert/rocon_msgs/blob/hydro-devel/scheduler_msgs/msg/AllocateResources.msg
.. _`scheduler_msgs/Request`: https://github.com/robotics-in-concert/rocon_msgs/blob/hydro-devel/scheduler_msgs/msg/Request.msg
.. _`scheduler_msgs/SchedulerFeedback`: https://github.com/robotics-in-concert/rocon_msgs/blob/hydro-devel/scheduler_msgs/msg/SchedulerFeedback.msg
.. _`Universally Unique Identifier`: http://en.wikipedia.org/wiki/Universally_unique_identifier
