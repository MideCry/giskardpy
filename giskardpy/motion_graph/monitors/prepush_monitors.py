from typing import Optional

from giskardpy import casadi_wrapper
from giskardpy.god_map import god_map
from giskardpy.motion_graph.monitors.monitors import ExpressionMonitor
import giskardpy.casadi_wrapper as cas
from giskardpy.motion_graph.tasks.task import WEIGHT_BELOW_CA
import numpy as np


class PrePushDoorMonitor(ExpressionMonitor):

    def __init__(self,
                 root_link: str,
                 tip_link: str,
                 door_object: str,
                 door_handle: str,
                 threshold: float,
                 root_group: Optional[str] = None,
                 tip_group: Optional[str] = None,
                 name: Optional[str] = None,
                 start_condition: cas.Expression = cas.TrueSymbol,
                 hold_condition: cas.Expression = cas.FalseSymbol,
                 end_condition: cas.Expression = cas.FalseSymbol):
        """
            The objective is to push the object until desired rotation is reached
        """
        self.root = god_map.world.search_for_link_name(root_link, root_group)
        self.tip = god_map.world.search_for_link_name(tip_link, tip_group)
        self.door_object = god_map.world.search_for_link_name(door_object)
        object_joint_name = god_map.world.get_movable_parent_joint(self.door_object)
        object_V_object_rotation_axis = cas.Vector3(god_map.world.get_joint(object_joint_name).axis)

        self.handle = god_map.world.search_for_link_name(door_handle)
        if name is None:
            name = f'{self.__class__.__name__}/{self.tip}/{self.door_object}'

        super().__init__(name=name,
                         start_condition=start_condition,
                         hold_condition=hold_condition,
                         end_condition=end_condition)

        root_T_tip = god_map.world.compose_fk_expression(self.root, self.tip)
        root_T_door = god_map.world.compose_fk_expression(self.root, self.door_object)
        door_P_handle = god_map.world.compute_fk_point(self.door_object, self.handle)
        temp_point = np.asarray([door_P_handle.x.to_np(), door_P_handle.y.to_np(), door_P_handle.z.to_np()])

        door_V_v1 = np.zeros(3)
        # axis pointing in the direction of handle frame from door joint frame
        direction_axis = np.argmax(abs(temp_point))
        door_V_v1[direction_axis] = 1
        door_V_v2 = object_V_object_rotation_axis  # B
        door_V_v1 = cas.Vector3(door_V_v1)  # A

        door_Pose_tip = god_map.world.compose_fk_expression(self.door_object, self.tip)
        door_P_tip = door_Pose_tip.to_position()
        dist, door_P_nearest = cas.distance_point_to_plane(door_P_tip,
                                                           door_V_v2,
                                                           door_V_v1)

        self.expression = cas.less(dist, threshold)
