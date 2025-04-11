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
                     force: float):
            super().__init__(run_call_in_thread=True,
                             name=name)
            self.force = force

        def __call__(self):
            goal = GripperApplyEffortGoal()
            goal.effort = self.force
            self.action_server.send_goal_and_wait(goal)
            self.state = ObservationState.true

        def get_state(self) -> ObservationState:
            return self.state


    class OpenHsrGripper(MoveHSRGripper):
        def __init__(self, *,
                     name: Optional[str] = None):
            super().__init__(name=name, force=0.8)


    class CloseHsrGripper(MoveHSRGripper):
        def __init__(self, *,
                     name: Optional[str] = None):
            super().__init__(name=name, force=-0.8)
