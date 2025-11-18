# params.py
from typing import Any

# ─── ROBOT CONSTANTS ──────────────────────────────────────────────────────────
# Common constants used across all robot sequences
SPEED_PRECISE = 10      # For precise operations
SPEED_CAREFUL = 25      # For careful handling  
SPEED_NORMAL = 50       # Normal operation speed
SPEED_FAST = 100        # Fast movements

# Common home positions used across multiple sequences
ESPRESSO_HOME = (42.159162, 16.269149, -135.156441, -81.822150, -49.784457, 13.771214)
ESPRESSO_GRINDER_HOME = (-32.837723, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)

GRIPPER_OPEN = 0        # Fully open gripper
GRIPPER_LIGHT = 100     # Light grip
GRIPPER_MEDIUM = 150    # Medium grip
GRIPPER_FIRM = 200      # Firm grip
GRIPPER_FULL = 255      # Maximum grip

# Common delays (in seconds)
DELAY_SHORT = 0.5       # Short delay between operations
DELAY_MEDIUM = 1.0      # Medium delay
DELAY_LONG = 2.0        # Long delay for settling

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
    'north':       (   0, 30, -130, -100,  -90,    0),
    'north_east': ( -45, 30, -130, -100,  -90,    0),
    'east':       ( -90, 30, -130, -100,  -90,    0),
    'south_east': (-135, 30, -130, -100,  -90,    0),
    'south':      ( 180, 30, -130, -100,  -90,    0),
    'south_west': ( 135, 30, -130, -100,  -90,    0),
    'west':       (  90, 30, -130, -100,  -90,    0),
    'north_west': (  45, 30, -130, -100,  -90,    0),
    'zero':       (   0,  0,    0,    0,     0,    0),
}

# ─── ESPRESSO PARAMETERS ───────────────────────────────────────────────────────────
# Note: ESPRESSO_HOME and ESPRESSO_GRINDER_HOME are defined at the top of this file (lines 12-13)

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
    'pos1':         (128.971283,-14.939140,-132.839478,-34.090870,-58.927341,-40.686066),
    'pour1':        (132.844193,-18.599159,-131.453491,-38.224197,-58.924160,-119.019859),
    'neutral1':     (128.971283,-14.939140,-132.839478,-34.090870,-58.927341,-40.686066),
    'pos2':         (145.129288,-20.563540,-123.503204,-38.293377,-42.786377,-39.922482),
    'pour2':        (147.791855,-24.902674,-120.071037,-45.219368,-44.186771,-115.968132),
    'neutral2':     (145.129288,-20.563540,-123.503204,-38.293377,-42.786377,-39.922482),
    'pos3':         (154.755386,-29.344767,-107.646370,-45.937229,-33.178307,-39.206394),
    'pour3':        (156.536041,-34.173431,-102.250099,-55.803558,-35.627476,-113.325684),
    'neutral3':     (154.755386,-29.344767,-107.646370,-45.937229,-33.178307,-39.206394),
    'pos4':         (160.584351,-40.439693,-86.204124,-56.838676,-27.368427,-38.566547),
    'pour4':        (161.838486,-46.053944,-78.337555,-69.684830,-30.483406,-111.122185),
    'neutral4':     (160.584351,-40.439693,-86.204124,-56.838676,-27.368427,-38.566547),
    'inter':        (121.242584,-10.634244,-135.326675,-34.000744,-58.927547,0.000000),
}

ESPRESSO_HOT_WATER_PARAMS = {
    'approach':     (67.341492, -49.798077, -99.061142, -30.809935, -22.613216, -0.457894),
    'position':     (61.759601, -52.680407, -91.236745, -35.814578, -28.197699, -0.388979),
    'retreat':      (67.341492, -49.798077, -99.061142, -30.809935, -22.613216, -0.457894),
}

# ─── MILK FROTHING PARAMETERS ─────────────────────────────────────────────────────
MILK_FROTHING_PARAMS = {
    'calibration': {
        'positioning': (-38.973099,-67.496948,-35.246922,-79.803886,-77.277245,9.584912),
    },
    'pickup': {
        'area':        (2.889473,-22.613735,-119.842255,-75.999672,-87.948189,-0.235808),
    },
    'mounting': {
        'prep':        (-4.127179, -41.282722, -129.513504, -21.285969, -62.760456, 7.227837),
    },
    'swirling': {
        'intermediate1': (-53.498047, -56.063831, -104.329971, -23.914228, -67.359390, 3.238193),
        'swirl_pos':    (-12.436070,-12.056064,-127.342203,-44.476541,-21.010899,4.820788),
    },
    'pouring': {
        'stage1': {
            'position':  (-96.038133,-16.779600,-104.255018,-62.483484,-81.154254,-37.125368),
            'adjust1':   (-80.742955,-20.628923,-107.592269,-56.447117,-75.887376,-108.203675),
        },
        'stage2': {
            'position':  (-110.625841,-24.341315,-92.985758,-66.165623,-95.718015,-38.024363),
            'adjust1':   (-99.369913,-24.209976,-97.719323,-62.612256,-94.456415,-109.700141),
        },
        'stage3': {
            'position':  (-122.820854,-36.136456,-73.316114,-74.204156,-107.900885,-38.813036),
            'adjust1':   (-113.296795,-33.403078,-82.373956,-68.997752,-108.344130,-110.861340),
        }, 
        'stage4': {
            'position':  (-132.368953,-52.469645,-42.776132,-88.685925,-117.453260,-39.514638),
            'adjust1':   (-125.360518,-47.083181,-57.256980,-80.924584,-120.379033,-112.033018),
        }, 
    },
    'return': {
        'intermediate': (-4.127179, -41.282722, -129.513504, -21.285969, -62.760456, 7.227837),
        'prep':         (22.402670, -79.481049, -59.269863, -39.324520, -81.417374, 11.115391),
    },
}

# ─── SLUSH PARAMETERS ─────────────────────────────────────────────────────────────
SLUSH_PARAMS = {
    'dispenser_1': {
        'approach':     (27.351225, -41.244622, -131.515709, -6.958192, -152.611468, 0.0),
        'dispense':     (63.173412, -59.329338, -104.639252, -17.729839, -112.244003, 0.0),
    },
    'dispenser_2': {
        'approach':     (27.351225, -41.244622, -131.515709, -6.958192, -152.611468, 0.0),
        'intermediate': (16.886827, -53.178346, -90.576631, -35.737992, -163.024408, 0.0),
        'dispense':     (45.045738, -70.610674, -67.497199, -43.970287, -130.385694, -0.683135),
    },
    'staging': {
        'stage_1':      (-111.215927, -19.601524, -91.144157, -68.881447, -114.195343, 0.046140),
        'stage_2':      (-121.922080, -29.170114, -76.511387, -73.910323, -124.911454, 0.116947),
    },
}

# ─── PLASTIC CUPS PARAMETERS ──────────────────────────────────────────────────────
PLASTIC_CUPS_PARAMS = {
    'dispenser': {
        'area':         (137.406860, 3.501065, -134.504471, -48.814426, -42.387501, -0.108438),
        '7oz_area':     (128.029709, 2.940671, -132.852890, -49.843658, -45.068768, -0.150343),
        '16oz_area':    (143.127869, -11.225266, -134.261658, -34.211472, -40.098595, -0.230271),
    },
    'staging': {
        'stage_1':      (-80.221687, -43.867016, -125.338081, -18.518541, -84.856163, 0.006812),
        'stage_2':      (-100.830803, -45.966148, -116.024010, -26.002304, -105.266800, -2.802466),
        'stage_3':      (-117.226875, -50.948524, -99.833191, -38.283516, -121.485474, -5.440053),
        'stage_4':      (-129.165802, -59.506020, -76.980766, -54.155602, -133.259628, -8.007045),
    },
    'ice_positions': {
        'approach':     (-34.285637, -96.143585, -85.111305, -52.668182, -77.955963, 40.874359),
        'dispense':     (-40.901531, -124.078323, -25.869335, -68.229462, -77.965225, 40.874393),
    },
    'gripper_settings': {
        '7oz': 145,
        '9oz': 145, 
        '12oz': 145,
        '16oz': 118,
    },
    'extraction_distances': {
        '7oz': -210,
        '9oz': -210,
        '12oz': -210,
        '16oz': -280,
    },
}

# ─── PAPER CUPS PARAMETERS ────────────────────────────────────────────────────────
GRAB_PAPER_CUP_PARAMS = {
    '12oz': {
        'twist_back':   (-142.260873, -17.875853,  10.033241,   8.226858,  -0.089241, -47.128327),
        'approach':     ( 231,        -5,          -5,          0,          0,         0),
        'grip_width':   150,
        'retreat':      (-350,        0,          0,          0,          0,         0),
    },
    '9oz': {
        'twist_back':   ( -92.886268, 7.237848, -6.756920, 3.452408, -1.172608, 0.187942),
        'approach':     ( 307.5,         -18,           5.0,          0,          0,         0),
        'grip_width':   160,
        'retreat':      (-300,         0,           0,          0,          0,         0),
    },
    '7oz': {
        'twist_back':   ( -65.440372, -10.652569,   4.188843,   6.867561,   0.095261,  29.626037),
        'approach':     ( 245.5,         0.0,           -9.0,          0,          0,         0),
        'grip_width':   187,
        'retreat':      (-290,         0,           0,          0,          0,         0),
    },
}

PLACE_PAPER_CUP_PARAMS = {
    'stage_1': {
        'twist':        (  66,   0,   0,   0,    0,    0),
        'pose':         ( 143.086101,-48.994568,-131.943897,1.363507,-36.809510,-0.385201),
        'stage_home':   (106.460129,  13.883821, -133.648376, -81.024788,  -49.533218,  13.894379),
        'twist_back':   (-64.032688,   0,        0,         0,         0,         0),
        'twist_serve':  ( 31.983258,   0,        0,         0,         0,         0),
        'pick':         ( 74.423628, -24.537379, -125.119809, -29.957357, -105.529320,   0.111478),
        'above_serve':  (114.123176, -17.424376, -139.079935, -23.095763,  -65.826354,  -0.154358),
        'serve':        (114.103052, -48.360801, -140.051323,   8.823533,  -65.854705,  -0.182434),
    },
    'stage_2': {
        'twist':        (  76,   0,   0,   0,    0,    0),
        'pose':         ( 154.082450,-50.373403,-117.476202,-11.568709,-25.815556,-0.552027),
        'stage_home':   (106.460129,  13.883821, -133.648376, -81.024788,  -49.533218,  13.894379),
        'twist_back':   (-64.032688,   0,        0,         0,         0,         0),
        'twist_serve':  ( 31.983258,   0,        0,         0,         0,         0),
        'pick':         ( 74.423628, -24.537379, -125.119809, -29.957357, -105.529320,   0.111478),
        'above_serve':  (114.123176, -17.424376, -139.079935, -23.095763,  -65.826354,  -0.154358),
        'serve':        (114.103052, -48.360801, -140.051323,   8.823533,  -65.854705,  -0.182434),
    },
    'stage_3': {
        'twist':        (  82,   0,   0,   0,    0,    0),
        'pose':         ( 160.501391,-55.374705,-97.871484,-26.003763,-19.406342,-0.726685),
        'stage_home':   (106.460129,  13.883821, -133.648376, -81.024788,  -49.533218,  13.894379),
        'twist_back':   (-64.032688,   0,        0,         0,         0,         0),
        'twist_serve':  ( 31.983258,   0,        0,         0,         0,         0),
        'pick':         ( 74.423628, -24.537379, -125.119809, -29.957357, -105.529320,   0.111478),
        'above_serve':  (114.123176, -17.424376, -139.079935, -23.095763,  -65.826354,  -0.154358),
        'serve':        (114.103052, -48.360801, -140.051323,   8.823533,  -65.854705,  -0.182434),
    },
    'stage_4': {
        'twist':        (  86,   0,   0,   0,    0,    0),
        'pose':         ( 164.522223,-64.404774,-72.245137,-42.425282,-15.404792,-0.905662),
        'stage_home':   (106.460129,  13.883821, -133.648376, -81.024788,  -49.533218,  13.894379),
        'twist_back':   (-64.032688,   0,        0,         0,         0,         0),
        'twist_serve':  ( 31.983258,   0,        0,         0,         0,         0),
        'pick':         ( 74.423628, -24.537379, -125.119809, -29.957357, -105.529320,   0.111478),
        'above_serve':  (114.123176, -17.424376, -139.079935, -23.095763,  -65.826354,  -0.154358),
        'serve':        (114.103052, -48.360801, -140.051323,   8.823533,  -65.854705,  -0.182434),
    },
}

# ─── CLEANING PARAMETERS ──────────────────────────────────────────────────────────
CLEANING_PARAMS = {
    'hard_brush_adjust': (-102.563631,-4.349989,-116.815596,-58.573670,-102.493498,-149.923394),
    'cleaning_motion_1': (0, 0, 0, 0, 0, 0),
    'cleaning_motion_2': (0, 0, 0.0, 0, 0, 0),
    'retreat_hard': (0, 0, 100, 0, 0, 0),
    'retreat_soft': (0, 0, 150, 0, 0, 0),
}

# ─── TEST PARAMETERS ──────────────────────────────────────────────────────────────
TEST_PARAMS = {
    'espresso_test_position': (42.159162, 16.269149, -135.156441, -81.822150, -49.784457, 13.771214),
    'test_cycles': 5,
    'cycle_delay': 0.5,
    'settling_delay': 0.25,
    'operational_delay': 0.6,
}