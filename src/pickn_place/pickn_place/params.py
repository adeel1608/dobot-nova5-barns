# params.py
from typing import Any

# ─── ROBOT CONSTANTS ──────────────────────────────────────────────────────────
# Common constants used across all robot sequences

# ── Speed Factors ──
SPEED_VERY_SLOW = 3      # Very slow for delicate operations
SPEED_SUPER_SLOW = 5     # Super slow for precise cup dispensing
SPEED_SLOW = 7           # Slow movements for cup extraction
SPEED_PRECISE_POURING = 9  # Precise speed for milk pouring
SPEED_PRECISE = 10       # For precise operations
SPEED_SLOW_POURING = 15  # Slow speed for pouring operations
SPEED_POUR_APPROACH = 20 # Approach speed for pouring
SPEED_CAREFUL = 25       # For careful handling  
SPEED_FROTHER_MOUNT = 40 # Speed for mounting frother
SPEED_NORMAL = 50        # Normal operation speed
SPEED_FAST = 100         # Fast movements

# ── Gripper Positions ──
GRIPPER_OPEN = 0         # Fully open gripper
GRIPPER_RELEASE_GENTLE = 10  # Gentle release for placing cups
GRIPPER_RELEASE = 25     # Standard release position
GRIPPER_HOLD_LOOSE = 75  # Loose hold for station placement
GRIPPER_RELEASE_PITCHER = 25  # Release position for espresso pitcher
GRIPPER_LIGHT = 100      # Light grip
GRIPPER_PITCHER_1 = 105  # Gripper setting for espresso pitcher port 2 & 3
GRIPPER_PITCHER_2 = 110  # Gripper setting for espresso pitcher port 1
GRIPPER_MEDIUM = 150     # Medium grip
GRIPPER_FROTHER_PICKUP = 169  # Initial frother pickup position
GRIPPER_FROTHER_RELEASE = 165  # Frother release position
GRIPPER_FIRM = 200       # Firm grip
GRIPPER_FROTHER_PLACE = 200  # Gripper setting for placing frother at milk station
GRIPPER_FULL = 255       # Maximum grip

# ── Common Delays (in seconds) ──
DELAY_VERY_SHORT = 0.5   # Very short delay
DELAY_SHORT = 1.0        # Short delay between operations
DELAY_MEDIUM = 2.0       # Medium delay for settling
DELAY_LONG = 2.5         # Long delay for pouring completion
DELAY_FROTHER_PICKUP = 5.0  # Delay for frother pickup stabilization

# ── Calibration Settings ──
CALIBRATION_APPROACH_CYCLES = 5  # Number of approach cycles for machine calibration
CALIBRATION_SETTLE_TIME = 1.0    # Settle time between calibration approaches

# Common home positions used across multiple sequences
ESPRESSO_HOME = (42.159162, 16.269149, -135.156441, -81.822150, -49.784457, 13.771214)
ESPRESSO_GRINDER_HOME = (-32.837723, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)

# Valid parameter values
VALID_PORTS = ('port_1', 'port_2', 'port_3')
VALID_CUP_POSITIONS = (1, 2, 3, 4)
VALID_CUP_POSITION_NAMES = ('cup_position_1', 'cup_position_2', 'cup_position_3', 'cup_position_4')
# Legacy stage names still supported internally
VALID_STAGES = ('1', '2', '3', '4')
VALID_STAGE_NAMES = ('stage_1', 'stage_2', 'stage_3', 'stage_4')
VALID_CUP_SIZES = ('7oz', '9oz', '12oz', '16oz')
VALID_DISPENSERS = ('1', '2')
VALID_HOME_POSITIONS = ('north', 'north_east', 'east', 'south_east', 
                       'south', 'south_west', 'west', 'north_west')

# Parameter defaults
DEFAULT_PORT = 'port_2'
DEFAULT_CUP_POSITION = 4  # Changed from 1 to test parameter passing
DEFAULT_STAGE = '4'  # Legacy support - changed to match
DEFAULT_CUP_SIZE = '12oz'
DEFAULT_PAPER_CUP_SIZE = '7oz'  # Default for paper cups (H-codes)
DEFAULT_PLASTIC_CUP_SIZE = '16oz'  # Default for plastic cups (C-codes)
DEFAULT_DISPENSER = '1'
DEFAULT_HOME = 'north'

# ─── STATE TRACKER FOR AUTO-DETECTION ────────────────────────────────────────
# Global state to track if we're coming from dispense_plastic_cup or go_home_with_ice
_cup_dispensed_flag = False

def _set_cup_dispensed():
    """Set flag indicating cup was just dispensed or came from ice."""
    global _cup_dispensed_flag
    _cup_dispensed_flag = True
    
def _check_and_clear_cup_dispensed() -> bool:
    """Check if cup was just dispensed, then clear the flag."""
    global _cup_dispensed_flag
    was_dispensed = _cup_dispensed_flag
    _cup_dispensed_flag = False  # Clear flag after checking
    return was_dispensed

# ─── HELPER FUNCTIONS ─────────────────────────────────────────────────────────
def _extract_cup_position(params: dict) -> int:
    """
    Extract cup_position from new parameter format.
    
    New format: {'position': {'cup_position': 1.0}}
    Also supports legacy: {'stage': '1'} or {'stage': 1}
    
    Returns:
        int: Cup position (1, 2, 3, or 4)
    """
    # Try new format first
    position_dict = params.get("position", {})
    if isinstance(position_dict, dict):
        cup_position = position_dict.get("cup_position")
        if cup_position is not None:
            try:
                pos = int(float(cup_position))
                if pos in (1, 2, 3, 4):
                    return pos
            except (ValueError, TypeError):
                pass
    
    # Fallback to legacy stage parameter for backward compatibility
    stage_value = params.get("stage")
    if stage_value is not None:
        try:
            if isinstance(stage_value, str) and stage_value.startswith("stage_"):
                pos = int(stage_value.split("_")[1])
            else:
                pos = int(float(stage_value))
            if pos in (1, 2, 3, 4):
                return pos
        except (ValueError, TypeError, IndexError):
            pass
    
    # Default to position 1
    return DEFAULT_CUP_POSITION

def _extract_cups_dict(params: dict) -> dict:
    """
    Extract cups dictionary from various parameter formats.
    
    Handles multiple input formats:
    - New nested format: {'ingredients': {'cups': {'cup_H12': 1.0}}}
    - Direct format: {'cups': {'cup_H12': 1.0}}
    - Array format: [{'ingredients': {'cups': ...}}]
    - Legacy format: {'size': '12oz'} or {'cup_size': '12oz'}
    
    Returns:
        dict: Cups dictionary or empty dict if not found
    """
    cups_dict = None
    
    # Try nested ingredients format first
    if 'ingredients' in params and isinstance(params['ingredients'], dict):
        cups_dict = params['ingredients'].get('cups')
        if cups_dict:
            return cups_dict if isinstance(cups_dict, dict) else {}
    
    # Try direct cups parameter
    cups_dict = params.get("cups")
    
    # Handle list format (array of cup items)
    if isinstance(cups_dict, list) and len(cups_dict) > 0:
        first_cup = cups_dict[0]
        if isinstance(first_cup, dict):
            # Check for nested ingredients
            if 'ingredients' in first_cup:
                cups_dict = first_cup['ingredients'].get('cups')
            # Check for direct size
            elif 'size' in first_cup:
                cups_dict = first_cup.get('size')
    
    # Fallback to legacy parameters
    if not cups_dict or not isinstance(cups_dict, dict):
        # Try old 'size' or 'cup_size' parameters
        size_param = params.get("size") or params.get("cup_size")
        if size_param:
            return {'legacy_size': size_param}  # Wrap for consistent handling
    
    return cups_dict if isinstance(cups_dict, dict) else {}

def _normalize_cup_size(cups_dict: Any, cup_type: str = 'paper', default_size: str = None) -> str:
    """
    Unified cup size normalizer for paper (H-codes) and plastic (C-codes) cups.
    
    Args:
        cups_dict: Dictionary containing cup information, or a simple string/value
        cup_type: Either 'paper' (H-codes: H7, H9, H12) or 'plastic' (C-codes: C7, C9, C12, C16)
        default_size: Default size to return if parsing fails (uses DEFAULT_PAPER_CUP_SIZE or DEFAULT_PLASTIC_CUP_SIZE if None)
        
    Returns:
        str: Normalized cup size (e.g., '7oz', '9oz', '12oz', '16oz')
        
    Example:
        >>> _normalize_cup_size({'cup_H12': 1.0}, 'paper')
        '12oz'
        >>> _normalize_cup_size({'cup_C16': 1.0}, 'plastic')
        '16oz'
    """
    # Set defaults based on cup type
    if default_size is None:
        default_size = DEFAULT_PAPER_CUP_SIZE if cup_type == 'paper' else DEFAULT_PLASTIC_CUP_SIZE
    
    # Define expected prefix and valid sizes
    expected_prefix = 'H' if cup_type == 'paper' else 'C'
    valid_sizes = {
        'paper': {'h7': '7oz', 'h9': '9oz', 'h12': '12oz', '7oz': '7oz', '9oz': '9oz', '12oz': '12oz'},
        'plastic': {'c7': '7oz', 'c9': '9oz', 'c12': '12oz', 'c16': '16oz', 
                   '7oz': '7oz', '9oz': '9oz', '12oz': '12oz', '16oz': '16oz'}
    }
    mapping = valid_sizes.get(cup_type, {})
    
    # Handle dictionary format
    if isinstance(cups_dict, dict):
        # Check for legacy_size wrapper
        if 'legacy_size' in cups_dict:
            size = str(cups_dict['legacy_size']).strip().upper()
        else:
            # Get the first key from the cups dictionary
            cup_key = next(iter(cups_dict.keys()), None)
            if not cup_key:
                return default_size
            
            # Extract cup code from key like 'cup_H12' -> 'H12'
            cup_key_str = str(cup_key).upper()
            if 'CUP_' in cup_key_str:
                cup_code = cup_key_str.split('CUP_', 1)[1]
            else:
                cup_code = cup_key_str
            
            # Validate prefix matches expected cup type
            if not cup_code.startswith(expected_prefix):
                return default_size
            
            size = cup_code
    else:
        # Backward compatibility: handle direct string/value
        if not cups_dict:
            return default_size
        size = str(cups_dict).strip().upper()
    
    # Normalize the size string
    normalized = str(size).strip().lower()
    return mapping.get(normalized, default_size)

def validate_port(port):
    """Validate port parameter"""
    if port not in VALID_PORTS:
        print(f"[ERROR] Invalid port: {port!r}. Valid ports: {', '.join(VALID_PORTS)}")
        return False
    return True

def validate_stage(stage):
    """Validate stage parameter"""
    if stage not in VALID_STAGES:
        print(f"[ERROR] Invalid stage: {stage!r}. Valid stages: {', '.join(VALID_STAGES)}")
        return False
    return True

def validate_cup_size(cup_size):
    """Validate cup size parameter"""
    if cup_size not in VALID_CUP_SIZES:
        print(f"[ERROR] Invalid cup size: {cup_size!r}. Valid sizes: {', '.join(VALID_CUP_SIZES)}")
        return False
    return True

def get_param_with_default(params, key, default):
    """Get parameter with default value"""
    return params.get(key, default) if params.get(key) is not None else default

def log_step(step_num, total_steps, description):
    """Log a formatted step"""
    print(f"📍 Step {step_num}/{total_steps}: {description}...")

def log_success(message, indent=0):
    """Log success message"""
    prefix = "   " * indent
    print(f"{prefix}✅ {message}")

def log_error(message, indent=0):
    """Log error message"""
    prefix = "   " * indent
    print(f"{prefix}❌ {message}")

def log_warning(message, indent=0):
    """Log warning message"""
    prefix = "   " * indent
    print(f"{prefix}⚠️  {message}")

def log_info(message, indent=0):
    """Log info message"""
    prefix = "   " * indent
    print(f"{prefix}ℹ️  {message}")

# ─── "HOME" POSE ANGLES ───────────────────────────────────────────────────────────
# Main-home is straight ahead; the compass points are ±45° increments
HOME_ANGLES = {
    'north':       (   0, 30, -130, -90,  -90,    0),
    'north_east': ( -45, 30, -130, -90,  -90,    0),
    'east':       ( -90, 30, -130, -90,  -90,    0),
    'south_east': (-135, 30, -130, -90,  -90,    0),
    'south':      ( 180, 30, -130, -90,  -90,    0),
    'south_west': ( 135, 30, -130, -90,  -90,    0),
    'west':       (  90, 30, -130, -90,  -90,    0),
    'north_west': (  45, 30, -130, -90,  -90,    0),
    'zero':       (   0,  0,    0,    0,     0,    0),
}

# ─── HOME CALIBRATION CONSTANTS ───────────────────────────────────────────────────
HOME_CALIBRATION_CONSTANTS = {
    'speed_factor': SPEED_FAST,  # Speed for calibration movements
    'approach_cycles': CALIBRATION_APPROACH_CYCLES,  # Number of approach cycles (5)
    'settle_time': CALIBRATION_SETTLE_TIME,  # Settle time between approaches (1.0s)
    'final_home_stagger': 1,  # Stagger time for final home movement (1s)
}

# ─── HOME CALIBRATION PARAMETERS ──────────────────────────────────────────────────
# Positions used in get_machine_position() for calibrating machine positions
HOME_CALIBRATION_PARAMS = {
    'return_home_position': (30.0, -130.0, -100.0, -90.0, 0.0),  # J2-J6 values for return_back_to_home (J1 is calculated)
    'portafilter_cleaner': {
        'prep_position': (-54.471272,-22.616722,-132.696136,-55.483162,-49.198364,24.286514),  # Preparation position for portafilter cleaner calibration
    },
    'espresso_grinder_calibration': {
        'prep1': (-62.837723, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360),  # First prep position for grinder calibration
        'prep2': (-44.767990, -16.740473, -125.801704, -51.932587, -94.604942, -0.214288),  # Second prep position for grinder calibration
    },
    'three_group_espresso_calibration': {
        'prep1': (7.427441, 13.883821, -133.648376, -81.024788, -49.533218, 13.894379),  # First prep position for three-group espresso calibration
    },
}

# ─── ESPRESSO CONSTANTS ───────────────────────────────────────────────────────────
ESPRESSO_SPEEDS = {
    'normal': SPEED_FAST,         # Normal espresso operation speed
    'pitcher_handling': SPEED_CAREFUL,  # Speed for handling espresso pitcher
    'hot_water_pour': SPEED_SLOW_POURING,  # Speed for hot water pouring
}

ESPRESSO_PITCHER_GRIPPER = {
    'port_1': GRIPPER_PITCHER_2,    # Gripper setting for port 1 pitcher (110)
    'port_2': GRIPPER_PITCHER_1,    # Gripper setting for port 2 pitcher (105)
    'port_3': GRIPPER_PITCHER_1,    # Gripper setting for port 3 pitcher (105)
    'release': GRIPPER_RELEASE_PITCHER,  # Release setting (75)
}

ESPRESSO_PORTAFILTER_GRIPPER = {
    'grip': GRIPPER_FULL,  # Full grip for portafilter operations
    'release': GRIPPER_OPEN,  # Release portafilter
}

ESPRESSO_MOVEMENT_OFFSETS = {
    'portafilter_clear_down': (0, 0, -35, 0, 0, 0),  # Move down to clear portafilter after unmount
    'portafilter_clear_up': (0, 0, 5, 0, 0, 0),      # Move up to fix portafilter during mount
    'hot_water_move': (-35, 0, 0, 0, 0, 0),          # Move for hot water positioning
    'hot_water_retreat': (-150, 0, 0, 0, 0, 0),      # Retreat after hot water
    'port_3_retreat': (-20, 0, 0, 0, 0, 0),          # Additional retreat for port 3
}

ESPRESSO_DELAYS = {
    'orientation_settle': 0.2,  # Delay after orientation enforcement
}

# Portafilter validation threshold (in millimeters)
PORTAFILTER_Z_THRESHOLD_MM = 10.0  # If Z difference > this, portafilter was filled twice

# ─── ESPRESSO PARAMETERS ───────────────────────────────────────────────────────────
# Note: ESPRESSO_HOME and ESPRESSO_GRINDER_HOME are defined at the top of this file

PULL_ESPRESSO_PARAMS = {
    'port_1': {
        'home':        ( 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379),
        'portafilter_number':         "portafilter_1",
        'group_number':         "group_1",
        'move_back':   (-5.932289,-9.177162,-138.612458,-46.645501,-94.418543,0.008893),
    },
    'port_2': {
        'home':        ( 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379),
        'portafilter_number':         "portafilter_2",
        'group_number':         "group_2",
        'move_back':   (88.717612,-29.575282,-135.766406,-12.283250,-5.212680,0.014052),
    },
    'port_3': {
        'home':        ( 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379),
        'portafilter_number':         "portafilter_3",
        'group_number':         "group_3",
        'move_back':   (88.717612,-29.575282,-135.766406,-12.283250,-5.212680,0.014052),
    },
}

ESPRESSO_GRINDER_PARAMS = {
    'nav1':         (57.162277, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360),
    'nav2':         (-32.837723, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360),
}

ESPRESSO_PITCHER_PARAMS = {
    'home':         (31.076585, -40.253154, -136.313679, -3.210446, -58.931112, -0.206957),
    'pos1':         (128.912586,-14.941652,-132.700460,-34.226286,-58.986065,-40.688192),
    'pour1':        (132.884964,-18.716054,-131.337356,-38.226868,-58.883868,-119.013201),
    'neutral1':     (128.912586,-14.941652,-132.700460,-34.226286,-58.986065,-40.688192),
    'pos2':         (145.035620,-20.577497,-123.353201,-38.415256,-42.880336,-39.934043),
    'pour2':        (147.768250,-25.031079,-119.905119,-45.241985,-44.204450,-115.980444),
    'neutral2':     (145.035620,-20.577497,-123.353201,-38.415256,-42.880336,-39.934043),
    'pos3':         (154.657353,-29.363111,-107.498584,-46.035256,-33.279858,-39.233358),
    'pour3':        (156.487541,-34.301428,-102.053874,-55.833863,-35.667728,-113.361040),
    'neutral3':     (154.657353,-29.363111,-107.498584,-46.035256,-33.279858,-39.233358),
    'pos4':         (160.494843,-40.463178,-86.054746,-56.912785,-27.468759,-38.612881),
    'pour4':        (161.786199,-46.189147,-78.100186,-69.724693,-30.530701,-111.181931),
    'neutral4':     (160.494843,-40.463178,-86.054746,-56.912785,-27.468759,-38.612881),
    'inter':        (121.242584,-10.634244,-135.326675,-34.000744,-58.927547,0.000000),
}

ESPRESSO_HOT_WATER_PARAMS = {
    'approach':     (67.341492, -49.798077, -99.061142, -30.809935, -22.613216, -0.457894),
    'position':     (61.759601, -52.680407, -91.236745, -35.814578, -28.197699, -0.388979),
    'retreat':      (67.341492, -49.798077, -99.061142, -30.809935, -22.613216, -0.457894),
}

# ─── MILK FROTHING CONSTANTS ──────────────────────────────────────────────────────
MILK_FROTHER_SPEEDS = {
    'calibration': SPEED_FAST,           # Speed for frother calibration
    'mount': SPEED_FROTHER_MOUNT,        # Speed for mounting frother to steam wand
    'swirl': SPEED_CAREFUL,              # Speed for swirling milk
    'pour_approach': SPEED_POUR_APPROACH,  # Speed for approaching pour position
    'pour': SPEED_PRECISE_POURING,       # Speed for pouring milk
    'return': SPEED_FAST,              # Speed for returning frother
}

MILK_FROTHER_GRIPPER_POSITIONS = {
    'open': GRIPPER_OPEN,                  # Fully open for positioning
    'pickup_initial': GRIPPER_FROTHER_PICKUP,  # Initial pickup position (169)
    'secure': GRIPPER_FULL,                # Secure grip on frother (255)
    'place': GRIPPER_FROTHER_PLACE,        # Place at milk station (200)
    'release': GRIPPER_FROTHER_RELEASE,    # Release frother (165)
}

MILK_FROTHER_MOVEMENT_OFFSETS = {
    'lift_after_place': (0, 0, 150, 0, 0, 0),  # Lift before placing frother at milk station
    'lift_after_pick': (0, 0, 10, 0, 0, 0),    # Lift after picking frother from milk station
    'cleaning_motion': (-25, 5, -150, 0, 0, 0),  # Cleaning motion offset
    'final_approach': (-75, 0, 0, 0, 0, 0),      # Final approach for frother return
}

MILK_POURING_OFFSETS = {
    'stage1': {'move_forward': (20, 0, 0, 0, 0, 0), 'move_up': (0, 0, 100, 0, 0, 0)},
    'stage2': {'move_forward': (25, 0, 0, 0, 0, 0), 'move_up': (0, 0, 100, 0, 0, 0)},
    'stage3': {'move_forward': (25, 0, 0, 0, 0, 0), 'move_up': (0, 0, 100, 0, 0, 0)},
    'stage4': {'move_forward': (25, 0, 0, 0, 0, 0), 'move_up': (0, 0, 100, 0, 0, 0)},
}

MILK_FROTHING_DELAYS = {
    'swirl_delay': DELAY_MEDIUM,          # Delay before swirling (2.0s)
    'pour_completion': DELAY_LONG,        # Delay for pour completion (3.0s)
    'frother_pickup': DELAY_FROTHER_PICKUP,  # Delay after frother pickup (5.0s)
    'frother_release': DELAY_VERY_SHORT,   # Delay after releasing frother (0.5s)
}

# Circle motion parameters for milk swirling
MILK_SWIRL_CIRCLE_PARAMS = {
    'cycles': 3,  # Number of circular motions
    'point1_offset': (-30.0, 0.0, 0.0, 0.0, 0.0, 0.0),  # First point offset
    'point2_offset': (-15.0, -15.0, 0.0, 0.0, 0.0, 0.0),  # Second point offset
    'options': ["tool=0"],  # Circle motion options
}

# Volume-based Z adjustment for frother
MILK_VOLUME_Z_ADJUSTMENT_FACTOR = 0.1866666667 * 0.7  # Factor for calculating Z adjustment based on milk volume

# ─── MILK FROTHING PARAMETERS ─────────────────────────────────────────────────────
MILK_FROTHING_PARAMS = {
    'calibration': {
        'positioning': (-50.017250,-75.956367,-20.588757,-82.413101,-108.227524,17.493626),  # Final positioning for frother calibration
    },
    'pickup': {
        'area':        (2.889473,-22.613735,-119.842255,-75.999672,-87.948189,-0.235808),  # Frother pickup area position
    },
    'milk_station': {
        'place_pre1':  (-6.988280,-50.951061,-132.741623,2.908280,-92.178726,8.730732),  # First pre-placement position at milk station
        'place_pre2':  (-58.038989,-65.359436,-78.628995,-38.342787,-141.938121,7.532277),  # Second pre-placement position at milk station
        'place_approach': (-40.325470,-71.810158,-65.020432,-39.404678,-117.536209,8.156257),  # Approach position for placing frother at milk station
        'place_final': (-40.189131,-72.847691,-64.628552,-39.734056,-117.229183,10.369100),  # Final placement position at milk station
        'pick_retreat1': (-40.325470,-71.810158,-65.020432,-39.404678,-117.536209,8.156257),  # First retreat position when picking from milk station
        'pick_retreat2': (-58.038989,-65.359436,-78.628995,-38.342787,-141.938121,7.532277),  # Second retreat position when picking from milk station
    },
    'mounting': {
        'prep':        (-4.127179, -41.282722, -129.513504, -21.285969, -62.760456, 7.227837),  # Preparation position for mounting frother
    },
    'swirling': {
        'intermediate1': (-53.498047, -56.063831, -104.329971, -23.914228, -67.359390, 3.238193),  # First intermediate position for swirling
        'swirl_pos':    (-12.436070,-12.056064,-127.342203,-44.476541,-21.010899,4.820788),  # Main swirling position
    },
    'pouring': {
        'stage1': {
            'position':  (-96.083982,-16.210451,-105.039001,-62.486753,-80.955525,-37.507634),  # Stage 1 pouring position
            'adjust1':   (-80.531382,-18.182229,-106.977193,-59.482663,-75.699375,-108.294767),  # Stage 1 pour angle adjustment
        },
        'stage2': {
            'position':  (-112.956263,-24.321635,-93.007182,-66.393067,-97.792107,-38.606995),  # Stage 2 pouring position
            'adjust1':   (-98.388205,-22.228447,-100.869531,-61.406274,-93.496126,-109.722007),  # Stage 2 pour angle adjustment
        },
        'stage3': {
            'position':  (-126.029066,-35.772446,-73.957985,-74.217283,-110.841058,-39.515233),  # Stage 3 pouring position
            'adjust1':   (-113.936546,-30.270324,-87.808810,-66.676734,-108.994360,-111.003200),  # Stage 3 pour angle adjustment
        }, 
        'stage4': {
            'position':  (-135.740098,-52.153730,-43.415727,-88.720604,-120.542044,-40.299402),  # Stage 4 pouring position
            'adjust1':   (-126.144187,-41.865918,-67.172854,-76.221819,-121.161589,-112.185484),  # Stage 4 pour angle adjustment
        }, 
    },
    'cleaning': {
        'pose1':       (-37.858528,-39.202564,-84.331383,-67.038254,-75.938263,-12.405199),  # First cleaning pose
        'pose2':       (-47.118893,-75.306686,-29.548725,-73.313492,-116.382469,4.306785),  # Second cleaning pose
        'pose3':       (-42.960231,-73.262903,-40.519041,-64.811101,-134.418391,-170.149991),  # Third cleaning pose
    },
    'return': {
        'intermediate': (-4.127179, -41.282722, -129.513504, -21.285969, -62.760456, 7.227837),  # Intermediate position for return
        'prep':         (22.402670, -79.481049, -59.269863, -39.324520, -81.417374, 11.115391),  # Preparation position for return
        'pre_return1':  (-42.453480,-74.396233,-37.945210,-66.263145,-133.914459,-170.167145),  # First pre-return position
        'pre_return2':  (-47.118893,-75.306686,-29.548725,-73.313492,-116.382469,4.306785),  # Second pre-return position (same as cleaning pose2)
        'pre_return3':  (-43.779022,-38.657257,-102.554436,-39.930614,-124.977203,4.300227),  # Third pre-return position
        'pre_return4':  (-0.401337,-55.195671,-129.519867,1.974099,-89.559532,4.300207),  # Fourth pre-return position
    },
}

# ─── SLUSH PARAMETERS ─────────────────────────────────────────────────────────────
SLUSH_PARAMS = {
    'navigation': {
        'intermediate': (106.212090, -43.618443, -136.693954, 1.223362, -23.919476, -0.124173),  # Intermediate position when navigating to slush area
        'slush_area':   (33.380177,-65.448544,-125.305906,18.179613,-139.723057,1.841451),  # Main slush dispensing area position
    },
    'dispenser_1': {
        'dispense':     (53.272518, -67.612831, -88.370926, -23.156694, -119.473190, -0.214796),  # Dispenser 1 dispensing position
        'retreat':      (45.785095, -64.636208, -119.745956, 10.442498, -127.393181, -0.156864),  # Retreat position from dispenser 1
    },
    'dispenser_2': {
        'intermediate': (17.117330,-72.397126,-55.180654,-48.179137,-153.839501,1.256867),  # Intermediate position for dispenser 2
        'dispense':     (39.080325,-80.227702,-48.030111,-51.277060,-130.866959,-0.148558),  # Dispenser 2 dispensing position
        'retreat':      (22.607694,-78.770437,-48.740179,-48.922226,-148.368363,0.479901),  # Retreat position from dispenser 2
    },
}

# ─── PLASTIC CUPS CONSTANTS ───────────────────────────────────────────────────────
PLASTIC_CUP_GRIPPER_POSITIONS = {
    '7oz': 145,   # Gripper position for 7oz plastic cup
    '9oz': 125,   # Gripper position for 9oz plastic cup (for station pickup)
    '12oz': 140,  # Gripper position for 12oz plastic cup (for station pickup)
    '16oz': 118,  # Gripper position for 16oz plastic cup
}

PLASTIC_CUP_DISPENSE_GRIPPER = {
    '7oz': 162,   # Gripper setting for 7oz cup dispenser
    '9oz': 150,   # Gripper setting for 9oz cup dispenser
    '12oz': 160,  # Gripper setting for 12oz cup dispenser
    '16oz': 140,  # Gripper setting for 16oz cup dispenser
}

PLASTIC_CUP_DISPENSE_SPEEDS = {
    '7oz': SPEED_SLOW,         # Speed for 7oz cup extraction
    '9oz': SPEED_VERY_SLOW,    # Speed for 9oz cup extraction
    '12oz': SPEED_SUPER_SLOW,  # Speed for 12oz cup extraction
    '16oz': SPEED_SLOW,        # Speed for 16oz cup extraction
}

PLASTIC_CUP_EXTRACT_OFFSETS = {
    '7oz': {'z1': -10.0, 'z2': -90},   # Z offsets for 7oz cup extraction
    '9oz': {'z1': -10, 'z2': -110},    # Z offsets for 9oz cup extraction
    '12oz': {'z1': -10, 'z2': -150},   # Z offsets for 12oz cup extraction
    '16oz': {'z1': -10, 'z2': -140},   # Z offsets for 16oz cup extraction
}

PLASTIC_CUP_MOVEMENT_OFFSETS = {
    'pickup_down': (0, -100, 0, 0, 0, 0),  # Move down to pickup cup from station
    'place_return_up': (0, 100, 0, 0, 0, 0),  # Move up when returning from placement
}

# ─── PLASTIC CUPS PARAMETERS ──────────────────────────────────────────────────────
PLASTIC_CUPS_PARAMS = {
    'dispenser': {
        # Cup dispenser coordinates for grabbing plastic cups
        '7oz_coords':   (157.029014,-29.678097,-117.343398,-32.416276,-22.856233,-0.528079),  # 7oz cup dispenser position
        '9oz_coords':   (145.520738,-21.027729,-128.794233,-29.809187,-34.358134,-0.313671),  # 9oz cup dispenser position
        '12oz_coords':  (115.363006,-26.809169,-138.267717,-14.690222,-64.514647,-0.132663),  # 12oz cup dispenser position
        '16oz_coords':  (79.037093,-39.242251,-132.377459,-8.192847,-100.849547,-0.020315),  # 16oz cup dispenser position
    },
    'ice_positions': {
        'position1':    (48.733238,-49.616558,-113.214279,-27.820314,-40.595863,0),  # First ice position (approach)
        'position2':    (46.241082,-71.126117,-82.426418,-26.093993,-43.687633,-0.302819),  # Second ice position (dispense)
    },
    'staging': {
        # Staging positions for placing plastic cups
        'place_1':      (-82.522181,-50.735762,-126.581344,-2.476677,-82.420135,-0.099928),  # Stage 1 placement position
        'place_2':      (-102.678188,-51.825704,-116.952115,-11.037622,-102.578374,-0.025293),  # Stage 2 placement position
        'place_3':      (-118.947162,-55.648503,-100.620319,-23.542009,-118.855164,0.038563),  # Stage 3 placement position
        'place_4':      (-130.494242,-62.993442,-78.390283,-38.407176,-130.418147,0.093833),  # Stage 4 placement position
        # Pickup positions for picking plastic cups (same as paper cups)
        'pickup_1':     (-79.183964,-53.698625,-144.678190,18.593209,-79.087779,-0.133025),  # Stage 1 pickup position
        'pickup_2':     (-107.022091,-50.755058,-132.327806,3.262599,-106.920207,-0.022566),  # Stage 2 pickup position
        'pickup_3':     (-126.092345,-52.392425,-113.869083,-13.545369,-125.993798,0.060536),  # Stage 3 pickup position
        'pickup_4':     (-137.939929,-58.575150,-91.108139,-30.090046,-137.854252,0.129489),  # Stage 4 pickup position
    },
    'sauces_station': {
        'position1':    (-38.902538,-62.473824,-116.293251,1.105230,-129.698776,-1.780196),  # First position at sauces station
        'position2':    (-27.222835,-66.184373,-95.733343,-17.900463,-117.142562,0.018696),  # Second position at sauces station (place/pick)
    },
    'milk_station': {
        'position1':    (-53.449154,-67.421219,-92.044746,-16.125631,-142.249084,0.477525),  # First position at milk station
        'position2':    (-38.663287,-75.415087,-67.047894,-37.337316,-128.603573,0.076145),  # Second position at milk station (place/pick)
    },
}

# ─── PAPER CUPS CONSTANTS ─────────────────────────────────────────────────────────
PAPER_CUP_GRIPPER_POSITIONS = {
    '7oz': 130,   # Gripper position for 7oz paper cup pickup from station
    '9oz': 135,   # Gripper position for 9oz paper cup pickup from station
    '12oz': 125,  # Gripper position for 12oz paper cup pickup from station
}

PAPER_CUP_MOVEMENT_OFFSETS = {
    'pickup_down': (0, -95, 0, 0, 0, 0),     # Move down to pickup cup from station
    'pickup_up': (0, 0, 200, 0, 0, 0),       # Move up after picking cup (using moveEE_movJ)
    'place_up': (0, 0, 150, 0, 0, 0),        # Move up after placing cup
    'place_return_up': (0, 100, 0, 0, 0, 0), # Move up when returning from placement
    'pickup_hot_water_down': (0, 100, 30, 0, 0, 0),     # Move down to pickup cup from station
}

# ─── PAPER CUPS PARAMETERS ────────────────────────────────────────────────────────
GRAB_PAPER_CUP_PARAMS = {
    '12oz': {
        'twist_back':   (-142.260873, -17.875853,  10.033241,   8.226858,  -0.089241, -47.128327),
        'approach':     ( 222.0,        -5,          0,          0,          0,         0),
        'grip_width':   145,
        'retreat':      (-350,        0,          0,          0,          0,         0),
    },
    '9oz': {
        'twist_back':   ( -92.886268, 7.237848, -6.756920, 3.452408, -1.172608, 0.187942),
        'approach':     ( 295,         -20,           10,          0,          0,         0),
        'grip_width':   160,
        'retreat':      (-300,         0,           0,          0,          0,         0),
    },
    '7oz': {
        'twist_back':   ( -65.440372, -10.652569,   4.188843,   6.867561,   0.095261,  29.626037),
        'approach':     ( 250,         -10,           0.0,          0,          0,         0),
        'grip_width':   155,
        'retreat':      (-290,         0,           0,          0,          0,         0),
    },
}

PAPER_CUPS_NAVIGATION_PARAMS = {
    'espresso_avoid':   (106.17209, 16.269149, -135.156441, -81.822150, -49.784457, 13.771214),  # Position to avoid hitting espresso machine
    'dispenser_area':   (120.389030, 22.860609, -73.526848, -39.810959, 90.144394, -154.586288),  # Paper cup dispenser area
    'twist_7oz':        (54.948658, 12.208040, -69.338005, -32.943398, 90.239655, -124.960251),  # Twist back position for 7oz
    'twist_9oz':        (27.502762, 30.098457, -80.283768, -36.358551, 88.971786, -154.398346),  # Twist back position for 9oz
    'twist_12oz':       (-21.871843, 4.984756, -63.493607, -31.584101, 90.055153, -201.714615),  # Twist back position for 12oz
    'intermediate':     (88.657143, 21.041538, -74.451630, -36.522381, 90.145508, -91.183128),  # Intermediate position for placement
    'twist_back_machine': (42.427441, 13.883821, -133.648376, -81.024788, -49.533218, 13.894379),  # Twist back towards machine
}

PLACE_PAPER_CUP_PARAMS = {
    'stage_1': {
        'twist':        (  66,   0,   0,   0,    0,    0),
        'pose':         (143.124736,-49.011046,-131.961019,1.450719,-36.804551,-0.410724),
        'stage_home':   (106.460129,  13.883821, -133.648376, -81.024788,  -49.533218,  13.894379),
        'twist_back':   (-64.032688,   0,        0,         0,         0,         0),
        'twist_serve':  ( 31.983258,   0,        0,         0,         0,         0),
        'pick':         ( 74.423628, -24.537379, -125.119809, -29.957357, -105.529320,   0.111478),
        'above_serve':  (114.123176, -17.424376, -139.079935, -23.095763,  -65.826354,  -0.154358),
        'serve':        (114.103052, -48.360801, -140.051323,   8.823533,  -65.854705,  -0.182434),
    },
    'stage_2': {
        'twist':        (  76,   0,   0,   0,    0,    0),
        'pose':         (154.109817,-50.383447,-117.491155,-11.470211,-25.821872,-0.600735),
        'stage_home':   (106.460129,  13.883821, -133.648376, -81.024788,  -49.533218,  13.894379),
        'twist_back':   (-64.032688,   0,        0,         0,         0,         0),
        'twist_serve':  ( 31.983258,   0,        0,         0,         0,         0),
        'pick':         ( 74.423628, -24.537379, -125.119809, -29.957357, -105.529320,   0.111478),
        'above_serve':  (114.123176, -17.424376, -139.079935, -23.095763,  -65.826354,  -0.154358),
        'serve':        (114.103052, -48.360801, -140.051323,   8.823533,  -65.854705,  -0.182434),
    },
    'stage_3': {
        'twist':        (  82,   0,   0,   0,    0,    0),
        'pose':         (160.521276,-55.378845,-97.891073,-25.884111,-19.420150,-0.799664),
        'stage_home':   (106.460129,  13.883821, -133.648376, -81.024788,  -49.533218,  13.894379),
        'twist_back':   (-64.032688,   0,        0,         0,         0,         0),
        'twist_serve':  ( 31.983258,   0,        0,         0,         0,         0),
        'pick':         ( 74.423628, -24.537379, -125.119809, -29.957357, -105.529320,   0.111478),
        'above_serve':  (114.123176, -17.424376, -139.079935, -23.095763,  -65.826354,  -0.154358),
        'serve':        (114.103052, -48.360801, -140.051323,   8.823533,  -65.854705,  -0.182434),
    },
    'stage_4': {
        'twist':        (  86,   0,   0,   0,    0,    0),
        'pose':         (164.537400,-64.400606,-72.277886,-42.277243,-15.423310,-1.003312),
        'stage_home':   (106.460129,  13.883821, -133.648376, -81.024788,  -49.533218,  13.894379),
        'twist_back':   (-64.032688,   0,        0,         0,         0,         0),
        'twist_serve':  ( 31.983258,   0,        0,         0,         0,         0),
        'pick':         ( 74.423628, -24.537379, -125.119809, -29.957357, -105.529320,   0.111478),
        'above_serve':  (114.123176, -17.424376, -139.079935, -23.095763,  -65.826354,  -0.154358),
        'serve':        (114.103052, -48.360801, -140.051323,   8.823533,  -65.854705,  -0.182434),
    },
}

PAPER_CUPS_STATION_PARAMS = {
    'staging': {
        # Placement positions (same as plastic cups)
        'place_1':      (-82.495347,-50.751238,-126.575974,-2.414609,-82.426553,-0.078193),  # Stage 1 placement position
        'place_2':      (-103.053575,-51.655540,-118.074840,-10.022299,-102.986149,0.016289),  # Stage 2 placement position
        'place_3':      (-119.620530,-54.959088,-103.132825,-21.640516,-119.557694,0.101475),  # Stage 3 placement position
        'place_4':      (-131.763914,-61.608221,-82.272711,-35.811243,-131.710665,0.182086),  # Stage 4 placement position
        # Pickup positions for paper cups
        'pickup_1':     (-79.151479,-53.722102,-144.665085,18.653262,-79.086572,-0.108530),  # Stage 1 pickup position
        'pickup_2':     (-107.618809,-50.837104,-133.238999,4.322921,-107.550365,0.027873),  # Stage 2 pickup position
        'pickup_3':     (-127.135882,-52.028920,-115.863508,-11.823941,-127.069217,0.139543),  # Stage 3 pickup position
        'pickup_4':     (-139.614437,-57.667847,-93.968886,-28.016244,-139.555665,0.242845),  # Stage 4 pickup position
        #Pickup positions for hot water
        'pickup_hot_water_1':     (163.559, -53.065, -148.063, 22.257, -16.378, -1.093),  # Stage 1 pickup position
        'pickup_hot_water_2':     (169.904, -49.016, -130.157, 0.890, -10.024, -1.687),  # Stage 2 pickup position
        'pickup_hot_water_3':     (172.801, -52.255, -108.986, -16.442, -7.133, -2.289),  # Stage 3 pickup position
        'pickup_hot_water_4':     (174.428, -60.166, -83.684, -33.228, -5.521, -2.896),  # Stage 4 pickup position
    },
    'milk_station': {
        'position1':    (-53.449154,-67.421219,-92.044746,-16.125631,-142.249084,0.477525),  # First position at milk station
        'position2':    (-38.513183,-75.295421,-64.619235,-39.884508,-128.454664,0.077704),  # Second position at milk station
        'position3':    (-38.663287,-75.415087,-67.047894,-37.337316,-128.603573,0.076145),  # Third position at milk station (place/pick)
    },
    'sauces_station': {
        'position1':    (-38.902538,-62.473824,-116.293251,1.105230,-129.698776,-1.780196),  # First position at sauces station
        'position2':    (-27.222216,-64.822159,-96.371883,-18.623413,-117.140324,0.020276),  # Second position at sauces station
        'position3':    (-27.222835,-66.184373,-95.733343,-17.900463,-117.142562,0.018696),  # Third position at sauces station (place/pick)
    },
}

# ─── CLEANING PARAMETERS ──────────────────────────────────────────────────────────
CLEANING_PARAMS = {
    'hard_brush_adjust': (-102.563631,-4.349989,-116.815596,-58.573670,-102.493498,-149.923394),  # Adjustment position for hard brush cleaning
    'retreat_hard': (0, 0, 100, 0, 0, 0),  # Retreat offset after hard brush
    'retreat_soft': (0, 0, 150, 0, 0, 0),  # Retreat offset after soft brush
}

# ─── TEST PARAMETERS ──────────────────────────────────────────────────────────────
TEST_PARAMS = {
    'espresso_test_position': (42.159162, 16.269149, -135.156441, -81.822150, -49.784457, 13.771214),
    'test_cycles': 5,
    'cycle_delay': 0.5,
    'settling_delay': 0.25,
    'operational_delay': 0.6,
}


# ─── EXPORT ALL ───────────────────────────────────────────────────────────────────
# Explicitly export all helper functions (including underscore-prefixed ones) and constants
__all__ = [
    # Helper functions (underscore-prefixed must be explicitly listed)
    '_set_cup_dispensed',
    '_check_and_clear_cup_dispensed',
    '_extract_cup_position',
    '_extract_cups_dict',
    '_normalize_cup_size',
    'validate_port',
    'validate_stage',
    'validate_cup_size',
    'get_param_with_default',
    'log_step',
    'log_success',
    'log_error',
    'log_warning',
    'log_info',
    # Speed constants
    'SPEED_VERY_SLOW', 'SPEED_SUPER_SLOW', 'SPEED_SLOW', 'SPEED_PRECISE_POURING',
    'SPEED_PRECISE', 'SPEED_SLOW_POURING', 'SPEED_POUR_APPROACH', 'SPEED_CAREFUL',
    'SPEED_FROTHER_MOUNT', 'SPEED_NORMAL', 'SPEED_FAST',
    # Gripper constants
    'GRIPPER_OPEN', 'GRIPPER_RELEASE_GENTLE', 'GRIPPER_RELEASE', 'GRIPPER_HOLD_LOOSE',
    'GRIPPER_RELEASE_PITCHER', 'GRIPPER_LIGHT', 'GRIPPER_PITCHER_1', 'GRIPPER_PITCHER_2',
    'GRIPPER_MEDIUM', 'GRIPPER_FROTHER_PICKUP', 'GRIPPER_FROTHER_RELEASE', 'GRIPPER_FIRM',
    'GRIPPER_FROTHER_PLACE', 'GRIPPER_FULL',
    # Delay constants
    'DELAY_VERY_SHORT', 'DELAY_SHORT', 'DELAY_MEDIUM', 'DELAY_LONG', 'DELAY_FROTHER_PICKUP',
    # Calibration constants
    'CALIBRATION_APPROACH_CYCLES', 'CALIBRATION_SETTLE_TIME',
    # Home positions
    'ESPRESSO_HOME', 'ESPRESSO_GRINDER_HOME', 'HOME_ANGLES',
    # Valid parameters
    'VALID_PORTS', 'VALID_CUP_POSITIONS', 'VALID_CUP_POSITION_NAMES', 'VALID_STAGES',
    'VALID_STAGE_NAMES', 'VALID_CUP_SIZES', 'VALID_DISPENSERS', 'VALID_HOME_POSITIONS',
    # Defaults
    'DEFAULT_PORT', 'DEFAULT_CUP_POSITION', 'DEFAULT_STAGE', 'DEFAULT_CUP_SIZE',
    'DEFAULT_PAPER_CUP_SIZE', 'DEFAULT_PLASTIC_CUP_SIZE', 'DEFAULT_DISPENSER', 'DEFAULT_HOME',
    # Parameter dictionaries
    'HOME_CALIBRATION_CONSTANTS', 'HOME_CALIBRATION_PARAMS',
    'ESPRESSO_SPEEDS', 'ESPRESSO_PITCHER_GRIPPER', 'ESPRESSO_PORTAFILTER_GRIPPER',
    'ESPRESSO_MOVEMENT_OFFSETS', 'ESPRESSO_DELAYS', 'PORTAFILTER_Z_THRESHOLD_MM',
    'PULL_ESPRESSO_PARAMS', 'ESPRESSO_GRINDER_PARAMS', 'ESPRESSO_PITCHER_PARAMS',
    'ESPRESSO_HOT_WATER_PARAMS',
    'MILK_FROTHER_SPEEDS', 'MILK_FROTHER_GRIPPER_POSITIONS', 'MILK_FROTHER_MOVEMENT_OFFSETS',
    'MILK_POURING_OFFSETS', 'MILK_FROTHING_DELAYS', 'MILK_SWIRL_CIRCLE_PARAMS',
    'MILK_VOLUME_Z_ADJUSTMENT_FACTOR', 'MILK_FROTHING_PARAMS',
    'SLUSH_PARAMS',
    'PLASTIC_CUP_GRIPPER_POSITIONS', 'PLASTIC_CUP_DISPENSE_GRIPPER',
    'PLASTIC_CUP_DISPENSE_SPEEDS', 'PLASTIC_CUP_EXTRACT_OFFSETS',
    'PLASTIC_CUP_MOVEMENT_OFFSETS', 'PLASTIC_CUPS_PARAMS',
    'PAPER_CUP_GRIPPER_POSITIONS', 'PAPER_CUP_MOVEMENT_OFFSETS',
    'GRAB_PAPER_CUP_PARAMS', 'PAPER_CUPS_NAVIGATION_PARAMS',
    'PLACE_PAPER_CUP_PARAMS', 'PAPER_CUPS_STATION_PARAMS',
    'CLEANING_PARAMS', 'TEST_PARAMS',
]
