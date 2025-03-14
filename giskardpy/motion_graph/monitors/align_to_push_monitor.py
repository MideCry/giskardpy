from typing import Optional

from giskardpy.data_types.data_types import ColorRGBA
from giskardpy.god_map import god_map
from giskardpy.motion_graph.monitors.feature_monitors import DistanceMonitor
from giskardpy.motion_graph.monitors.monitors import ExpressionMonitor
import giskardpy.casadi_wrapper as cas
import numpy as np

from giskardpy.motion_graph.tasks.task import WEIGHT_BELOW_CA


class AlignToPushDoorMonitor(ExpressionMonitor):

    def __init__(self,
                 root_link: str,
                 tip_link: str,
                 door_object: str,
                 door_handle: str,
                 tip_gripper_axis: cas.Vector3,
                 goal_angle: Optional[float] = None,
                 root_group: Optional[str] = None,
                 tip_group: Optional[str] = None,
                 reference_linear_velocity: float = 0.1,
                 reference_angular_velocity: float = 0.5,
                 intermediate_point_scale: float = 1,
                 weight: float = WEIGHT_BELOW_CA,
                 name: Optional[str] = None,
                 threshold: float = 0.02,
                 start_condition: cas.Expression = cas.TrueSymbol,
                 hold_condition: cas.Expression = cas.FalseSymbol,
                 end_condition: cas.Expression = cas.FalseSymbol):
        """
        The objective is to reach an intermediate point before pushing the door
        """
        self.root = god_map.world.search_for_link_name(root_link, root_group)
        self.tip = god_map.world.search_for_link_name(tip_link, tip_group)
        self.handle = god_map.world.search_for_link_name(door_handle)
        self.door_object = god_map.world.search_for_link_name(door_object)
        self.reference_linear_velocity = reference_linear_velocity
        self.reference_angular_velocity = reference_angular_velocity
        self.weight = weight

        if name is None:
            name = f'{self.__class__.__name__}/{self.root}/{self.tip}'
        super().__init__(name=name,
                         start_condition=start_condition,
                         hold_condition=hold_condition,
                         end_condition=end_condition)

        object_joint_name = god_map.world.get_movable_parent_joint(self.door_object)
        object_joint_angle = god_map.world.state[object_joint_name].position

        tip_gripper_axis.scale(1)
        self.tip_gripper_axis = tip_gripper_axis
        object_V_object_rotation_axis = cas.Vector3(god_map.world.get_joint(object_joint_name).axis)

        root_T_door_expr = god_map.world.compose_fk_expression(self.root, self.door_object)
        door_P_handle = god_map.world.compute_fk_point(self.door_object, self.handle)
        temp_point = np.asarray([door_P_handle.x.to_np(), door_P_handle.y.to_np(), door_P_handle.z.to_np()])
        door_P_intermediate_point = np.zeros(3)
        # axis pointing in the direction of handle frame from door joint frame
        direction_axis = np.argmax(abs(temp_point))
        door_P_intermediate_point[direction_axis] = temp_point[direction_axis]*intermediate_point_scale
        door_P_intermediate_point = cas.Point3([door_P_intermediate_point[0],
                                                door_P_intermediate_point[1],
                                                door_P_intermediate_point[2]], reference_frame=door_P_handle.reference_frame)

        # # point w.r.t door
        if goal_angle is None:
            desired_angle = object_joint_angle * 0.5  # just chose 1/2 of the goal angle
        else:
            desired_angle = goal_angle * 0.7

        # find point w.r.t rotated door in local frame
        door_R_door_rotated = cas.RotationMatrix.from_axis_angle(axis=object_V_object_rotation_axis,
                                                                 angle=desired_angle)
        door_T_door_rotated = cas.TransMatrix(door_R_door_rotated)
        # as the root_T_door is already pointing to a completely rotated door, we invert desired angle to get to the
        # intermediate point
        door_rotated_P_top = cas.dot(door_T_door_rotated.inverse(), door_P_intermediate_point)
        root_P_top = cas.dot(cas.TransMatrix(root_T_door_expr), door_rotated_P_top)
        root_P_top.reference_frame = self.root

        tip_point = cas.Point3()
        tip_point.reference_frame = self.tip
        root_T_tip = god_map.world.compose_fk_expression(root_link=self.root, tip_link=self.tip)
        tip_point = root_T_tip.dot(tip_point)

        god_map.debug_expression_manager.add_debug_expression('tip_point', tip_point,
                                                              color=ColorRGBA(0.5, 0.5, 0.5, 1))

        dist = cas.norm(root_P_top-tip_point)
        god_map.debug_expression_manager.add_debug_expression('dist', dist,
                                                              color=ColorRGBA(0.5, 0.5, 0.5, 1))

        self.expression = cas.less(dist, threshold)
