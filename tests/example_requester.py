#!/usr/bin/env python
""" Requester usage example. """
import rospy
from scheduler_msgs.msg import Request, Resource
from rocon_scheduler_requests.requester import Requester


class ExampleRequester:

    def __init__(self):
        rospy.init_node("example_requester")
        self.rqr = Requester(self.feedback)
        self.request_turtlebot()
        self.timer = rospy.Timer(rospy.Duration(2.0), self.periodic_update)
        rospy.spin()

    def feedback(self, rset):
        """ Scheduler feedback function. """
        for rq in rset.values():
            if rq.msg.status == Request.WAITING:
                rospy.loginfo('Request queued: ' + str(rq.get_uuid()))
            elif rq.msg.status == Request.GRANTED:
                rospy.loginfo('Request granted: ' + str(rq.get_uuid()))
            elif rq.msg.status == Request.RELEASED:
                rospy.loginfo('Request canceled: ' + str(rq.get_uuid()))

    def periodic_update(self, event):
        """ Timer event handler for periodic request updates. """
        try:
            # cancel the previously-issued request
            self.rqr.rset[self.request_id].cancel()
        except KeyError:
            # previous request is gone, request another similar robot
            self.request_turtlebot()

    def request_turtlebot(self):
        """ Request any tutlebot able to run *example_rapp*. """
        bot = Resource(name='example_rapp',
                       platform_info='*.*.ros.turtlebot.*')
        self.request_id = self.rqr.new_request([bot])
        self.rqr.send_requests()

if __name__ == '__main__':
    node = ExampleRequester()
