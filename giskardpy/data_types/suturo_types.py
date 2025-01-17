from enum import unique, Enum


# States for the HSRs gripper, is being used only in old_python_interface as of now
@unique
class GripperTypes(Enum):
    OPEN = 'open'
    CLOSE = 'close'
    NEUTRAL = 'neutral'


# List of Methods that need Thresholds for force_monitor: GraspObjectCarefully(might include Doors?), Placing
# TODO: Rework Enum Names
@unique
class ForceTorqueThresholds(Enum):
    GRASP = 1
    PLACE = 2
    DOOR = 3
    DISHDOOR = 4
    FT_Tilt = 5  # Pouring (not currently in use)


# List of Objects that need to be differentiated between when placing method is used
@unique
class ObjectTypes(Enum):
    OT_Default = 'Default'  # Normal Objects(e.g Milk), includes Cups/Glasses, since planning grabs them from front
    OT_Cutlery = 'Cutlery'
    OT_Plate = 'Plate'
    OT_Bowl = 'Bowl'
    OT_Tray = 'Tray'


# List of possible grasping directions
@unique
class GraspTypes(Enum):
    FRONT = 'front'
    ABOVE = 'above'
    LEFT = 'left'
    RIGHT = 'right'
    BELOW = 'below'
