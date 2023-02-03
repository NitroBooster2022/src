#!/usr/bin/env python3
import rospy
from message_filters import TimeSynchronizer
from std_msgs.msg import String
from pystate import State, Event, Machine
from control.msg import Lane, Sign

class LaneFollowing(State):
    @stop_sign_event.on_enter
    def stop_sign_action(self):
        self.decision = self.machine.stop_sign_decision # this decision should be obtained from the localization node
        if self.decision == "stop":
            self.machine.stop_event()
        else:
            self.machine.timeout_event()

    @timeout_event.on_enter
    def timeout_action(self):
        self.machine.intersection_maneuvering_event()

    @intersection_maneuvering_event.on_enter
    def intersection_maneuvering_action(self):
        self.decision = self.machine.intersection_decision # this decision should be obtained from the localization node
        if self.decision == "right":
            self.machine.right_event()
        elif self.decision == "left":
            self.machine.left_event()
        else:
            self.machine.straight_event()

class Stop(State):
    def on_enter(self, n):
        self.n = n
        self.timer = threading.Timer(n, self.timeout_event.set)
        self.timer.start()
    def on_exit(self):
        self.timer.cancel()
    def timeout_event(self):
        self.machine.timeout_event()

class IntersectionManeuvering(State):
    pass

# Define the actions
def stop_sign_action(event, source, dest, lane, sign, drivable):
    if sign.type == "stop":
        # Stop the car for 3 seconds
        pass
    else:
        event.machine.stop()
def timeout_action(event, source):
    pass
def callback(lane, sign):
    # Perform image processing tasks
    # Compute the steering angle
    # Publish the steering angle to the /automobile/command topic
    # Update the state machine with the new information
    machine.update(lane=lane, sign=sign)
    # Perform the action corresponding to the current state
    machine.act()

if __name__ == '__main__':
    rospy.init_node('main_node')
    # Subscribe to topics
    # image_sub = rospy.Subscriber('/image_raw', String, queue_size=3)
    lane_sub = rospy.Subscriber('lane', Lane, queue_size=3)
    sign_sub = rospy.Subscriber('sign', Sign, queue_size=3)

    # Create an instance of TimeSynchronizer
    ts = TimeSynchronizer([lane_sub, sign_sub], 10)
    ts.registerCallback(callback)

    # Create the state machine
    # Create the state machine
    machine = Machine(
    states=[LaneFollowing, Stop, IntersectionManeuvering],
    events=[stop_sign_event, timeout_event],
    initial=LaneFollowing
)

    # Define the states
    driving_state = State("driving")
    stopping_state = State("stopping")

    # Define the events
    stop_sign_event = Event("stop_sign", source=stopping_state, dest=driving_state)
    timeout_event = Event("timeout", source=stopping_state)

    rate = rospy.Rate(10) # 10 Hz
    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()