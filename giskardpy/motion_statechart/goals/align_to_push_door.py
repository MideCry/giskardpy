from typing import Optional

import numpy as np

from giskardpy import casadi_wrapper as cas
from giskardpy.data_types.data_types import ColorRGBA
from giskardpy.god_map import god_map
from giskardpy.motion_statechart.goals.goal import Goal
from giskardpy.motion_statechart.tasks.task import WEIGHT_BELOW_CA, Task


class AlignToPushDoor(Goal):

    def __init__(self,
                 root_link: str,
                 tip_link: str,
                 door_object: str,
                 door_handle: str,
                 tip_gripper_axis: cas.Vector3,
                 goal_angle: Optional[float] = None,
                 root_group: Optional[str] = None,
                 tip_group: Optional[str] = None,
                 distance_threshold: float = 0.01,
                 angle_threshold: float = 0.01,
                 reference_linear_velocity: float = 0.1,
                 reference_angular_velocity: float = 0.5,
                 intermediate_point_scale: float = 1,
                 weight: float = WEIGHT_BELOW_CA,
                 name: Optional[str] = None):
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

        object_joint_name = god_map.world.get_movable_parent_joint(self.door_object)
        object_joint_angle = god_map.world.state[object_joint_name].position

        tip_gripper_axis.scale(1)
        self.tip_gripper_axis = tip_gripper_axis
        object_V_object_rotation_axis = cas.Vector3(god_map.world.get_joint(object_joint_name).axis)
        joint_limit = god_map.world.compute_joint_limits(object_joint_name, 0)

        if name is None:
            name = f'{self.__class__.__name__}/{self.root}/{self.tip}'
        super().__init__(name=name)

        root_T_tip = god_map.world.compose_fk_expression(self.root, self.tip)
        root_T_door_expr = god_map.world.compose_fk_expression(self.root, self.door_object)
        tip_V_tip_grasp_axis = cas.Vector3(self.tip_gripper_axis)
        root_V_object_rotation_axis = cas.dot(root_T_door_expr, object_V_object_rotation_axis)
        root_V_tip_grasp_axis = cas.dot(root_T_tip, tip_V_tip_grasp_axis)
        door_P_handle = god_map.world.compute_fk_point(self.door_object, self.handle)
        temp_point = np.asarray([door_P_handle.x.to_np(), door_P_handle.y.to_np(), door_P_handle.z.to_np()])
        door_P_intermediate_point = np.zeros(3)
        # axis pointing in the direction of handle frame from door joint frame
        direction_axis = np.argmax(abs(temp_point))
        door_P_intermediate_point[direction_axis] = temp_point[direction_axis] * intermediate_point_scale
        door_P_intermediate_point = cas.Point3([door_P_intermediate_point[0],
                                                door_P_intermediate_point[1],
                                                door_P_intermediate_point[2]])

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

        god_map.debug_expression_manager.add_debug_expression('goal_point', root_P_top,
                                                              color=ColorRGBA(0, 0.5, 0.5, 1))
        god_map.debug_expression_manager.add_debug_expression('root_V_grasp_axis', root_V_tip_grasp_axis,
                                                              color=ColorRGBA(0, 0, 1, 1))
        god_map.debug_expression_manager.add_debug_expression('root_V_object_axis', root_V_object_rotation_axis,
                                                              color=ColorRGBA(1, 0, 0, 1))
        align_to_push_task = Task(name='align_to_push_door')
        self.add_task(align_to_push_task)
        align_to_push_task.add_point_goal_constraints(frame_P_current=root_T_tip.to_position(),
                                                      frame_P_goal=root_P_top,
                                                      reference_velocity=self.reference_linear_velocity,
                                                      weight=self.weight)

        align_to_push_task.add_vector_goal_constraints(frame_V_current=root_V_tip_grasp_axis,
                                                       frame_V_goal=root_V_object_rotation_axis,
                                                       reference_velocity=self.reference_angular_velocity,
                                                       weight=self.weight)

        dist = cas.euclidean_distance(root_T_tip.to_position(), root_P_top)
        angle = cas.angle_between_vector(root_V_tip_grasp_axis, root_V_object_rotation_axis)
        self.observation_expression = cas.logic_and(
            cas.less_equal(dist, distance_threshold),
            cas.less_equal(angle, angle_threshold)
        )
