import os
from copy import copy
from typing import Optional

from rospy import wait_for_message

from giskardpy.middleware import get_middleware
from giskardpy.symbol_manager import symbol_manager

if 'GITHUB_WORKFLOW' not in os.environ:
    pass
import rospy
from geometry_msgs.msg import WrenchStamped

import giskardpy.casadi_wrapper as cas
from giskardpy.god_map import god_map
from giskardpy.motion_graph.monitors.monitors import PayloadMonitor
from giskardpy.data_types.suturo_types import ForceTorqueThresholds, ObjectTypes


class PayloadForceTorque(PayloadMonitor):
    def __init__(self,
                 # threshold_enum is needed here for the class to be able to handle the suturo_types appropriately
                 threshold_enum: int,
                 topic: str,
                 # object_type is needed to differentiate between objects with different thresholds
                 # (not needed for door, just pass an empty string)
                 object_type: Optional[str] = None,
                 name: Optional[str] = None,
                 start_condition: cas.Expression = cas.TrueSymbol,
                 hold_condition: cas.Expression = cas.FalseSymbol,
                 end_condition: cas.Expression = cas.FalseSymbol,
                 stay_true: bool = True):
        """
        The PayloadForceTorque class creates a monitor for the usage of the HSRs Force-Torque Sensor.
        This makes it possible for goals which use the Force-Torque Sensor to be used with Monitors,
        specifically to end/hold a goal automatically when a certain Force/Torque Threshold is being surpassed.

        :param threshold_enum: contains the enum of the threshold that will be used (normally an action e.g. PLACE)
        :param object_type: is used to determine the type of object that is being placed, is left empty if no object is being placed
        :param topic: the name of the topic
        :param name: name of the monitor class
        :param start_condition: the start condition of the monitor
        :param hold_condition: the hold condition of the monitor
        :param end_condition: the end condition of the monitor
        """

        super().__init__(name=name, start_condition=start_condition, run_call_in_thread=False)
        self.object_type = object_type
        self.threshold_enum = threshold_enum
        self.topic = topic
        self.wrench = WrenchStamped()
        self.bf = god_map.world.search_for_link_name('base_footprint')
        self.sensor_frame = god_map.world.search_for_link_name(wait_for_message(topic, WrenchStamped).header.frame_id)
        self.subscriber = rospy.Subscriber(name=topic,
                                           data_class=WrenchStamped, callback=self.cb)
        self.strategy = ThresholdStrategyFactory.get_strategy(self.object_type, self.threshold_enum)

    def cb(self, data: WrenchStamped):
        self.rob_force, self.rob_torque = self.force_T_base_transform(data)

    def force_T_base_transform(self, wrench):
        """
        The force_T_base_transform method is used to transform the Vector data from the
        force-torque sensor frame into the HSRs base frame, so that the axis stay
        the same, to ensure that the threshold check is actually done on the correct axis,
        since conversion into the map frame can lead to different values depending on how the map is recorded.

        :param wrench: the wrench of the WrenchStamped, used to get header and force-torque data
        """
        wrench.header.frame_id = self.sensor_frame

        vstampF = cas.Vector3.from_xyz(wrench.wrench.force.x, wrench.wrench.force.y, wrench.wrench.force.z,
                                       wrench.header.frame_id)
        vstampT = cas.Vector3.from_xyz(wrench.wrench.torque.x, wrench.wrench.torque.y, wrench.wrench.torque.z,
                                       wrench.header.frame_id)

        force_transformed = symbol_manager.evaluate_expr(god_map.world.transform(self.bf, vstampF))

        torque_transformed = symbol_manager.evaluate_expr(god_map.world.transform(self.bf, vstampT))

        # print("Force:", force_transformed.vector.x, force_transformed.vector.y, force_transformed.vector.z)
        # print("Torque:", torque_transformed.vector.x, torque_transformed.vector.y, torque_transformed.vector.z)

        return force_transformed, torque_transformed

    def __call__(self):
        rob_force = copy(self.rob_force)
        rob_torque = copy(self.rob_torque)
        # if self.state is necessary here because otherwise the monitor will return false
        # in next iteration after already having returned True thus always cancelling the current goal
        if self.state:
            return

        if self.strategy.check_thresholds(rob_force, rob_torque):
            self.state = True
        else:
            self.state = False

        # if self.threshold_name == ForceTorqueThresholds.GRASP.value:
        #     print(rob_force)
        #     print(self.state)
        #
        #     # case for grasping "normal" objects (namely Milk, Cereal and cups)
        #     if self.object_type == ObjectTypes.OT_Default.value:
        #
        #         torque_threshold = 2
        #
        #         if abs(rob_torque[1]) > torque_threshold:
        #             self.state = True
        #             get_middleware().loginfo(f'HIT GWC: TORQUE_Y:{rob_torque[1]}')
        #         else:
        #             self.state = False
        #
        #     # case for grasping cutlery
        #     elif self.object_type == ObjectTypes.OT_Cutlery.value:
        #
        #         force_threshold = 85
        #
        #         if abs(rob_force[2]) > force_threshold:
        #             self.state = True
        #             get_middleware().loginfo(f'HIT GWC: FORCE_Z:{rob_torque[2]}')
        #         else:
        #             self.state = False
        #
        #     # case for grasping plate
        #     # NOT CURRENTLY USED AS PLATES ARE NEITHER PLACED NOR PICKED UP
        #     elif self.object_type == ObjectTypes.OT_Plate.value:
        #
        #         torque_threshold = 0.02
        #
        #         if (abs(rob_force[1]) > torque_threshold or
        #                 abs(rob_force[1]) > torque_threshold):
        #             self.state = False
        #             print(f'HIT GWC: {rob_force[0]};{rob_torque[1]}')
        #         else:
        #             self.state = True
        #             raise Exception("HSR failed to Grasp Object, Grasping threshold has been Undershot.")
        #
        #     # case for grasping Bowl
        #     elif self.object_type == ObjectTypes.OT_Bowl.value:
        #
        #         force_threshold = 50.0
        #
        #         if abs(rob_force[2]) > force_threshold:
        #             self.state = True
        #             print(rob_force[2])
        #             print(self.state)
        #             get_middleware().loginfo(rob_force[2])
        #         else:
        #             self.state = False
        #
        #     # if no valid object_type has been declared in method parameters
        #     else:
        #         raise Exception("No valid object_type found, unable to determine placing thresholds!")
        #
        # # TODO: Add thresholds and cases for other object types
        # elif self.threshold_name == ForceTorqueThresholds.PLACE.value:
        #
        #     # case for placing "normal" objects (namely Milk, Cereal and cups)
        #     if self.object_type == ObjectTypes.OT_Default.value:
        #
        #         force_z_threshold = 35
        #
        #         if abs(rob_force[2]) >= force_z_threshold:
        #
        #             self.state = True
        #             get_middleware().loginfo(
        #                 f'HIT PLACING!: Z:{rob_force[2]}')
        #         else:
        #             self.state = False
        #
        #     # case for placing cutlery
        #     elif self.object_type == ObjectTypes.OT_Cutlery.value:
        #
        #         force_z_threshold = 35
        #
        #         if abs(rob_force[2]) >= force_z_threshold:
        #
        #             self.state = True
        #             get_middleware().loginfo(
        #                 f'HIT CUTLERY!: Z:{rob_force[2]}')
        #         else:
        #             self.state = False
        #
        #     # case for placing plates
        #     # NOT CURRENTLY USED AS PLATES ARE NEITHER PLACED NOR PICKED UP
        #     elif self.object_type == ObjectTypes.OT_Plate.value:
        #         #  TODO: Add proper placing logic for Plate
        #         force_z_threshold = 1.0
        #
        #         if abs(rob_force[2]) >= force_z_threshold:
        #
        #             self.state = True
        #             print(f'HIT PLACING: X:{rob_force[0]};Z:{rob_force[2]};Y:{rob_force[1]}')
        #         else:
        #             self.state = False
        #             print(f'MISS PLACING!: X:{rob_force[0]};Z:{rob_force[2]};Y:{rob_force[1]}')
        #
        #     # case for placing bowls
        #     elif self.object_type == ObjectTypes.OT_Bowl.value:
        #
        #         force_z_threshold = 35
        #
        #         if abs(rob_force[2]) >= force_z_threshold:
        #
        #             self.state = True
        #             get_middleware().loginfo(
        #                 f'HIT PLACING: Z:{rob_force[2]}')
        #         else:
        #             self.state = False
        #     # if no valid object_type has been declared in method parameters
        #     else:
        #         raise Exception("No valid object_type found, unable to determine placing thresholds!")
        #
        # else:
        #     raise Exception("No valid threshold_name found, unable to determine proper course of action!")


class ThresholdStrategy:
    def check_thresholds(self, rob_force, rob_torque):
        raise NotImplementedError("This method should be overridden.")


class GraspThresholdStrategy(ThresholdStrategy):
    def __init__(self, object_type):
        self.object_type = object_type

    def check_thresholds(self, rob_force, rob_torque):
        # Implement the logic for checking thresholds in GRASP context case for grasping "normal" objects
        # (namely Milk, Cereal and cups)
        if self.object_type == ObjectTypes.OT_Default.value:

            torque_threshold = 2

            if abs(rob_torque[1]) > torque_threshold:
                get_middleware().loginfo(f'HIT GWC: TORQUE_Y:{rob_torque[1]}')
                return True
            else:
                return False

        # case for grasping cutlery
        elif self.object_type == ObjectTypes.OT_Cutlery.value:

            force_threshold = 85

            if abs(rob_force[2]) > force_threshold:
                get_middleware().loginfo(f'HIT GWC: FORCE_Z:{rob_torque[2]}')
                return True
            else:
                return False

        # case for grasping plate
        # NOT CURRENTLY USED AS PLATES ARE NEITHER PLACED NOR PICKED UP DUE TO HSRs GRIPPER
        elif self.object_type == ObjectTypes.OT_Plate.value:

            torque_threshold = 0.02

            if (abs(rob_force[1]) > torque_threshold or
                    abs(rob_force[1]) > torque_threshold):
                print(f'HIT GWC: {rob_force[0]};{rob_torque[1]}')
                return False
            else:
                return True

        # case for grasping Bowl
        elif self.object_type == ObjectTypes.OT_Bowl.value:

            force_threshold = 50.0

            if abs(rob_force[2]) > force_threshold:
                print(rob_force[2])
                get_middleware().loginfo(rob_force[2])
                return True
            else:
                return False

        # if no valid object_type has been declared in method parameters
        else:
            raise Exception("No valid object_type found, unable to determine placing thresholds!")


class PlaceThresholdStrategy(ThresholdStrategy):
    def __init__(self, object_type):
        self.object_type = object_type

    def check_thresholds(self, rob_force, rob_torque):

        # case for placing "normal" objects (namely Milk, Cereal and cups)
        if self.object_type == ObjectTypes.OT_Default.value:

            force_z_threshold = 35

            if abs(rob_force[2]) >= force_z_threshold:

                get_middleware().loginfo(
                    f'HIT PLACING!: Z:{rob_force[2]}')
                return True
            else:
                return False

        # case for placing cutlery
        elif self.object_type == ObjectTypes.OT_Cutlery.value:

            force_z_threshold = 35

            if abs(rob_force[2]) >= force_z_threshold:

                get_middleware().loginfo(
                    f'HIT CUTLERY!: Z:{rob_force[2]}')
                return True
            else:
                return False

        # case for placing plates
        # NOT CURRENTLY USED AS PLATES ARE NEITHER PLACED NOR PICKED UP
        elif self.object_type == ObjectTypes.OT_Plate.value:
            #  TODO: Add proper placing logic for Plate
            force_z_threshold = 1.0

            if abs(rob_force[2]) >= force_z_threshold:

                print(f'HIT PLACING: X:{rob_force[0]};Z:{rob_force[2]};Y:{rob_force[1]}')
                return True
            else:
                print(f'MISS PLACING!: X:{rob_force[0]};Z:{rob_force[2]};Y:{rob_force[1]}')
                return False

        # case for placing bowls
        elif self.object_type == ObjectTypes.OT_Bowl.value:

            force_z_threshold = 35

            if abs(rob_force[2]) >= force_z_threshold:

                get_middleware().loginfo(
                    f'HIT PLACING: Z:{rob_force[2]}')
                return True
            else:
                return False
        # if no valid object_type has been declared in method parameters
        else:
            raise Exception("No valid object_type found, unable to determine placing thresholds!")


class DoorThresholdStrategy(ThresholdStrategy):

    def check_thresholds(self, rob_force, rob_torque):
        force_z_threshold = 70

        if abs(rob_force[2]) >= force_z_threshold:
            get_middleware().loginfo(
                f'HIT DOOR!: X:{rob_force[2]}')
            return True
        else:
            return False


class ThresholdStrategyFactory:
    """
    The ThresholdStrategyFactory takes the given threshold and object type
    and calls the appropriate ThresholdStrategy, which in turn possesses the
    logic for the different object types.
    """

    @staticmethod
    def get_strategy(object_type, threshold_enum):
        if threshold_enum == ForceTorqueThresholds.GRASP.value:
            return GraspThresholdStrategy(object_type)

        elif threshold_enum == ForceTorqueThresholds.PLACE.value:
            return PlaceThresholdStrategy(object_type)

        elif threshold_enum == ForceTorqueThresholds.DOOR.value:
            return DoorThresholdStrategy()
        else:
            raise ValueError(f"Invalid threshold name: {threshold_enum}")
