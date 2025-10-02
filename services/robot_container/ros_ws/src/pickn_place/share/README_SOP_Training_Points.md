============================================================================
                    SOP: ROBOT TRAINING POINTS CALIBRATION
============================================================================

OVERVIEW
--------
This document outlines the Standard Operating Procedure (SOP) for training and 
calibrating all required points for the BARNS robotic coffee system. The system 
requires multiple types of calibration points to operate accurately.


TRAINING POINT CATEGORIES
=========================

┌─ 1. TOOL OFFSET POINTS (tool_offset_points.yaml) ─────────────────────────┐
│ These points define the robot's interaction with various tools and        │
│ attachments.                                                              │
└───────────────────────────────────────────────────────────────────────────┘

   REQUIRED POINTS:
   
   ◆ Double Portafilter (ID: 12)
     ├─ approach_pose: Pre-grasp positioning
     └─ grab_pose: Final grasp position
     
   ◆ Single Portafilter (ID: 11) 
     ├─ approach_pose: Pre-grasp positioning
     └─ grab_pose: Final grasp position

   ◆ Left Steam Wand (ID: 18)
     ├─ approach_pose: Pre-grasp positioning
     └─ grab_pose: Final grasp position

   ◆ Milk Frother 1 (ID: 13)
     ├─ approach_pose: Pre-grasp positioning
     └─ grab_pose: Final grasp position

   ◆ Milk Frother 2 (ID: 14)
     ├─ approach_pose: Pre-grasp positioning
     └─ grab_pose: Final grasp position


┌─ 2. MACHINE OFFSET POINTS (machine_offset_points.yaml) ───────────────────┐
│ These points define robot interaction with coffee machine components.     │
└───────────────────────────────────────────────────────────────────────────┘

   ◆ Espresso Grinder (ID: 31):
     ├─ grinder.approach_pose: Safe approach to grinder
     ├─ grinder.mount_pose: Above the tamper
     ├─ tamper.approach_pose: Pushing the grinder button
     └─ tamper.mount_pose: Into the tamper

   ◆ Three Group Espresso Machine (ID: 41):
     ├─ espresso_machine_home: Home/safe position
     ├─ hot_water: Hot water dispenser position
     ├─ pick_pitcher_1/2/3: Pitcher pickup positions (IDs: 15, 16, 17)
     └─ portafilter_1/2/3: Portafilter mounting positions

   ◆ Left Steam Wand:
     ├─ deep_froth.approach_pose: Deep frothing approach position
     └─ deep_froth.mount_pose: Deep frothing mounting position

   ◆ Portafilter Cleaner (ID: 23):
     ├─ hard_brush: Hard brush cleaning positions
     └─ soft_brush: Soft brush cleaning positions


TRAINING PROCEDURE
==================

┌─ PHASE 1: VISION SYSTEM CALIBRATION ──────────────────────────────────────┐
│                                                                           │
│   1. Setup ArUco Markers                                                  │
│      • Place calibration tags (IDs 1-4) in workspace                      │
│      • Ensure proper lighting and marker visibility                       │
│      • Verify marker sizes match configuration (60mm for calibration)     │
│                                                                           │
│   2. Hand-Eye Calibration                                                 │
│      • Use calibration tags for AX=XB procedure                           │
│      • Record transformation between camera and robot base                │
│      • Validate calibration accuracy                                      │
│                                                                           │
└───────────────────────────────────────────────────────────────────────────┘

┌─ PHASE 2: TOOL TRAINING ──────────────────────────────────────────────────┐
│                                                                           │
│   1. Manual Teaching                                                      │
│      • Make robot grab the tool how its supposed to in operating procedure│
│      • Run the tool teach node                                            │
│                                                                           │
└───────────────────────────────────────────────────────────────────────────┘

┌─ PHASE 3: MACHINE INTEGRATION ────────────────────────────────────────────┐
│                                                                           │
│   1. Equipment Positioning                                                │
│      • Train all machine interaction points                               │
│      • Include approach and mount poses where applicable                  │
│      • Run machine mount node                                             │
│      • Verify operational clearances                                      │
│                                                                           │
│   2. Workflow Testing                                                     │
│      • Test complete sequences (e.g., grind → tamp → brew)                │
│      • Validate position accuracy under load                              │
│      • Adjust for thermal expansion and wear                              │
│                                                                           │
└───────────────────────────────────────────────────────────────────────────┘

============================================================================