### Unit tests
#
#   Only run when CATKIN_ENABLE_TESTING is true.

##  Python
#   Unit test uses nose, but needs rostest to create a ROS environment.
add_rostest(py_reqsched.test)

#catkin_add_nosetests(test_scheduler_request_manager.py)
