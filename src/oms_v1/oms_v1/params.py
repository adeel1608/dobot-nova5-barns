# params.py
Espresso_home = (42.427441, 13.883821, -133.648376, -81.024788, -49.533218, 13.894379)
Espresso_grinder_home = (-32.837723, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)
# ─── “HOME” POSE ANGLES ───────────────────────────────────────────────────────────
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
}
# ─── CUP GRAB PARAMETERS ────────────────────────────────────────────────────────
# twist_back: joint angles to back off before approach
# approach:   end-effector offsets to move in
# grip_width: how wide to open the gripper
# retreat:    end-effector offsets to back away
GRAB_CUP_PARAMS = {
    '12oz': {
        'twist_back':   (-142.260873, -17.875853,  10.033241,   8.226858,  -0.089241, -47.128327),
        'approach':     ( 200,        0,          0,          0,          0,         0),
        'grip_width':   139,
        'retreat':      (-350,        0,          0,          0,          0,         0),
    },
    '9oz': {
        'twist_back':   ( -94.810739,  -3.781208,   0.171860,   4.133488,  -0.141411,   0.198394),
        'approach':     ( 284,         5,           -10,          0,          0,         0),
        'grip_width':   144,
        'retreat':      (-300,         0,           0,          0,          0,         0),
    },
    '7oz': {
        'twist_back':   ( -65.440372, -10.652569,   4.188843,   6.867561,   0.095261,  29.626037),
        'approach':     ( 245,         15,           -10,          0,          0,         0),
        'grip_width':   140,
        'retreat':      (-300,         0,           0,          0,          0,         0),
    },
}
# ─── CUP PLACEMENT / SERVE PARAMETERS ────────────────────────────────────────────
# twist:       initial joint-1 twist into staging area
# pose:        joint targets to move into place over cup
# stage_home:  safe “staging” home pose
# twist_back:  untwist back towards machine
# twist_serve: joint-1 twist to present cup
# pick/above_serve/serve:
#   intermediate poses for the serve_cup() routine
PLACE_CUP_PARAMS = {
    'stage_1': {
        'twist':        (  66,   0,   0,   0,    0,    0),
        'pose':         ( 154.125778,-52.670326,-116.881065,-10.357515,-25.897715,0.057296),
        'stage_home':   (106.460129,  13.883821, -133.648376, -81.024788,  -49.533218,  13.894379),
        'twist_back':   (-64.032688,   0,        0,         0,         0,         0),
        'twist_serve':  ( 31.983258,   0,        0,         0,         0,         0),
        'pick':         ( 74.423628, -24.537379, -125.119809, -29.957357, -105.529320,   0.111478),
        'above_serve':  (114.123176, -17.424376, -139.079935, -23.095763,  -65.826354,  -0.154358),
        'serve':        (114.103052, -48.360801, -140.051323,   8.823533,  -65.854705,  -0.182434),
    },
    'stage_2': {
        # identical to stage_1 for now; tweak if you need a different trajectory
        'twist':        (  69,   0,   0,   0,    0,    0),
        'pose':         ( 159.539737,-56.095770,-101.541641,-22.208326,-20.521090,0.048888),
        'stage_home':   (106.460129,  13.883821, -133.648376, -81.024788,  -49.533218,  13.894379),
        'twist_back':   (-64.032688,   0,        0,         0,         0,         0),
        'twist_serve':  ( 31.983258,   0,        0,         0,         0,         0),
        'pick':         ( 74.423628, -24.537379, -125.119809, -29.957357, -105.529320,   0.111478),
        'above_serve':  (114.123176, -17.424376, -139.079935, -23.095763,  -65.826354,  -0.154358),
        'serve':        (114.103052, -48.360801, -140.051323,   8.823533,  -65.854705,  -0.182434),
    },
}
# ─── PULL PARAMETERS FOR ESPRESSO ──────────────────────────────────────────────
PULL_ESPRESSO_PARAMS = {
    'port_1': {
        'home':        ( 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379),
        'portafilter_number':         "portafilter_1",
        'group_number':         "group_1",
        'below_port':   (23.433992,-34.688675,-112.293900,-33.728462,-24.236135,0.000000),
        'move_back':   (-5.931169,-7.389562,-137.818634,-49.226776,-94.418251,0.010815),
    },
    'port_2': {
        'home':        ( 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379),
        'portafilter_number':         "portafilter_2",
        'group_number':         "group_2",
        'below_port':   (48.987930,-42.926491,-94.531250,-42.794937,4.182595,0.000000),
        'move_back':   (88.718765,-27.610367,-135.543320,-14.489676,-5.211227,0.032554),
    },
    'port_3': {
        'home':        ( 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379),
        'portafilter_number':         "portafilter_3",
        'group_number':         "group_3",
        'below_port':   (63.598370,-58.609657,-61.130093,-60.226448,15.355281,0.000000),
        'move_back':   (88.718765,-27.610367,-135.543320,-14.489676,-5.211227,0.032554),
    },
}

# ─── POUR PARAMETERS FOR ESPRESSO ──────────────────────────────────────────────
# Each entry holds joint targets for a midpoint above the cup and the cup position itself.
POUR_PARAMS = {
    'cup_1': {
        'cup':      (85.997716, -17.821510, -132.407788, -29.390120, -73.953046, -0.093618),
    },
    'cup_2': {
        'cup':      (85.997716, -17.821510, -132.407788, -29.390120, -73.953046, -0.093618),
    },
}
