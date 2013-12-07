### Unit tests
#
#   Only run when CATKIN_ENABLE_TESTING is true.

##  Python

# Unit tests not needing a running ROS core.
catkin_add_nosetests(test_common.py)

# Unit tests using nose, but needing a running ROS core.
# (needs better test programs)
#add_rostest(py_reqsched.test)
