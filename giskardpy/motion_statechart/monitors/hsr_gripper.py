import os

from giskardpy.data_types.data_types import ObservationState

if 'GITHUB_WORKFLOW' not in os.environ:
    from typing import Optional

    import actionlib

    from giskardpy import casadi_wrapper as cas
    from giskardpy.motion_statechart.monitors.monitors import PayloadMonitor

    from tmc_control_msgs.msg import GripperApplyEffortAction, GripperApplyEffortGoal


    class MoveHSRGripper(PayloadMonitor):
        action_server: actionlib.SimpleActionClient = actionlib.SimpleActionClient('/hsrb/gripper_controller/grasp',
                                                                                   GripperApplyEffortAction)

        def __init__(self, *,
                     name: Optional[str] = None,
                     force: float,
                     start_condition: cas.Expression = cas.BinaryTrue):
            super().__init__(run_call_in_thread=True,
                             name=name,
                             start_condition=start_condition)
            self.force = force

        def __call__(self):
            goal = GripperApplyEffortGoal()
            goal.effort = self.force
            self.action_server.send_goal_and_wait(goal)
            self.state = ObservationState.true


    class OpenHsrGripper(MoveHSRGripper):
        def __init__(self, *,
                     name: Optional[str] = None,
                     start_condition: cas.Expression = cas.BinaryTrue,
                     pause_condition: cas.Expression = cas.BinaryFalse,
                     end_condition: cas.Expression = cas.BinaryFalse):
            super().__init__(name=name, force=0.8, start_condition=start_condition)


    class CloseHsrGripper(MoveHSRGripper):
        def __init__(self, *,
                     name: Optional[str] = None,
                     start_condition: cas.Expression = cas.BinaryTrue,
                     pause_condition: cas.Expression = cas.BinaryFalse,
                     end_condition: cas.Expression = cas.BinaryFalse):
            super().__init__(name=name, force=-0.8, start_condition=start_condition)
