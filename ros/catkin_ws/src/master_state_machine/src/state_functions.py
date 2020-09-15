import time
import numpy
from smach import State, StateMachine, Concurrence
from smach_ros import SimpleActionState
from base_controller.msg import GotoPointAction, GotoPointGoal, GotoPoseAction, GotoPoseGoal, ResetOdomAction
from lifter_controller.msg import GrabAction, GrabGoal, LiftAction, LiftGoal
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point, Quaternion
from ball_mapper.srv import ClosestBall, ClosestBallRequest
from smach_ros import ServiceState

def build_reset_state():
    sm = StateMachine(outcomes=["succeeded", "preempted", "aborted"])

    with sm:
        StateMachine.add("RESET_ODOM", reset_odom(), {"succeeded": "OPEN_LEFT"})
        StateMachine.add("OPEN_LEFT", release("left_grabber"), {"succeeded": "OPEN_RIGHT"})
        StateMachine.add("OPEN_RIGHT", release("right_grabber"), {"succeeded": "LOWER_LIFTER"})
        StateMachine.add("LOWER_LIFTER", lower(), {"succeeded": "succeeded"})
    return sm

def build_bucket_drop_state():
    sm = StateMachine(outcomes=["succeeded", "preempted", "aborted"])

    with sm:
        StateMachine.add("LINE_FRONT_FIELD", drive_pose(0.661, 0.833, 90, 0.01, 1), {"succeeded": "DELAY"})
        StateMachine.add("DELAY", Delay(10), {"succeeded": "DRIVE_LEFT_BUCKET_FIELD"})
        StateMachine.add("DRIVE_LEFT_BUCKET_FIELD", drive_pose(0.4, 1.8, 90, 0.2, 20), {"succeeded": "LIFT_LIFTER"})
        StateMachine.add("LIFT_LIFTER", lift(), {"succeeded": "DRIVE_LEFT_BUCKET"})
        StateMachine.add("DRIVE_LEFT_BUCKET", drive_pose(0.17, 1.99, 90, 0.01, 5), {"succeeded": "OPEN_LEFT_GRABBER"})
        StateMachine.add("OPEN_LEFT_GRABBER", release("left_grabber"), {"succeeded": "DRIVE_LEFT_BUCKET_TURN"})
        StateMachine.add("DRIVE_LEFT_BUCKET_TURN", drive_pose(0.17, 1.99, 117.5, 0.01, 5), {"succeeded": "OPEN_RIGHT_GRABBER"})
        StateMachine.add("OPEN_RIGHT_GRABBER", release("right_grabber"), {"succeeded": "LINE_FIELD"})
        StateMachine.add("LINE_FIELD", drive_pose(0.7, 1.6, -90, 0.2, 10), {"succeeded": "LOWER_LIFTER"})
        StateMachine.add("LOWER_LIFTER", lower())
    return sm

def build_ball_timeout_state(grabber_frame, distance_limit, timeout):
    sm = Concurrence(outcomes=["succeeded", "timed_out"],
                    default_outcome="timed_out",
                    output_keys=["ball_location"],
                    outcome_map={"succeeded":{"CLOSEST_BALL": "succeeded"}})

    with sm:
        Concurrence.add("CLOSEST_BALL", "",
                        remapping={"position": "ball_location"})
        Concurrence.add("TIMEOUT", Delay(timeout))
    return sm

def build_ball_retry_state(grabber_frame, distance_limit):
    sm = StateMachine(outcomes=["succeeded", "preempted", "aborted"],
                        output_keys=["ball_location"])
    with sm:
        StateMachine.add("DETECT_CLOSEST_BALL", closest_ball(grabber_frame, distance_limit),
            {"succeeded": "succeeded", "aborted": "DETECT_CLOSEST_BALL"},
            remapping={"position": "ball_location"})
    return sm

class Delay(State):
    def __init__(self, delay_time):
        State.__init__(self, outcomes=["succeeded"])
        self.delay_time = delay_time

    def execute(self, userdata):
        time.sleep(self.delay_time)
        return 'succeeded'

def build_pickup_state(grabber_frame, distance_limit):
    sm = StateMachine(outcomes=["succeeded", "preempted", "aborted"])

    with sm:
        def drive_ball_goal_cb(userdata, goal, frame, distance_threshold):
            goto_goal = GotoPointGoal()
            goto_goal.point = userdata.ball_location
            goto_goal.target_frame = "odom"
            goto_goal.reference_frame = data=frame
            goto_goal.distance_threshold = distance_threshold
            return goto_goal

        StateMachine.add("DETECT_CLOSEST_BALL", build_ball_retry_state(grabber_frame, distance_limit),
                {"succeeded": "DRIVE_BALL_STAGE"})
        StateMachine.add("DRIVE_BALL_STAGE", SimpleActionState('goto_point', GotoPointAction,
                goal_cb=lambda userdata, goal: drive_ball_goal_cb(userdata, goal, grabber_frame + "_stage", 0.03),
                input_keys=["ball_location"]),
                {"succeeded": "DRIVE_BALL"})
        StateMachine.add("DRIVE_BALL", SimpleActionState('goto_point', GotoPointAction,
                goal_cb=lambda userdata, goal: drive_ball_goal_cb(userdata, goal, grabber_frame, 0.01),
                input_keys=["ball_location"]),
                {"succeeded": "CLOSE_GRABBER"})
        StateMachine.add("CLOSE_GRABBER", SimpleActionState('grab', GrabAction,
                goal=GrabGoal(grabber_frame=grabber_frame, position=GrabGoal.CLOSE)),
                {"succeeded": "succeeded"})
    return sm

def drive_point(x, y, distance_threshold):
    goal = GotoPointGoal()
    goal.point = Point(x, y, 0)
    goal.distance_threshold = distance_threshold
    goal.target_frame = "map"
    goal.reference_frame = "base_footprint"

    return SimpleActionState('goto_point', GotoPointAction, goal=goal)

def drive_pose(x, y, angle, distance_threshold, rotation_threshold):
    quat = quaternion_from_euler(0, 0, numpy.radians(angle))

    goal = GotoPoseGoal()
    goal.point = Point(x, y, 0)
    goal.distance_threshold = distance_threshold
    goal.rotation = Quaternion(*quat)
    goal.rotation_threshold = numpy.radians(rotation_threshold)
    goal.target_frame = "map"
    goal.reference_frame = "base_footprint"

    return SimpleActionState('goto_pose', GotoPoseAction, goal=goal)

def grab(frame):
    return SimpleActionState('grab', GrabAction,
                goal=GrabGoal(grabber_frame=frame, position=GrabGoal.CLOSE))

def release(frame):
    return SimpleActionState('grab', GrabAction,
                goal=GrabGoal(grabber_frame=frame, position=GrabGoal.OPEN))

def lift():
    return SimpleActionState('lift', LiftAction, goal=LiftGoal(position=LiftGoal.UP))

def lower():
    return SimpleActionState('lift', LiftAction, goal=LiftGoal(position=LiftGoal.DOWN))

def reset_odom():
    return SimpleActionState('reset_odom', ResetOdomAction)

def closest_ball(grabber_frame, distance_limit):
    return ServiceState('ball_map/closest', ClosestBall,
            request=ClosestBallRequest(grabber_frame=grabber_frame, distance_limit=distance_limit),
            response_slots=["position"])
