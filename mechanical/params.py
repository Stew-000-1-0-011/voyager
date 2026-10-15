"""All dimensions (mm). Coordinates: origin on the floor under the frame centre,
+X forward, +Y left, +Z up. Values marked (measure) must be checked on the real part."""

# frame (20mm aluminium extrusion)
FRAME_W = 280.0          # along Y (sonar cans included the closed width stays <= 320)
FRAME_D = 250.0          # along X
EXTRUSION = 20.0
FRAME_Z = 30.0           # underside of the extrusions above the floor

# wheels / drive motors (JGA25-370)
WHEEL_D = 65.0
WHEEL_W = 26.0
WHEEL_X = 30.0           # wheel centre ahead of the frame centre
MOTOR_DIA = 25.0         # gearbox diameter (measure)
MOTOR_LEN = 52.0         # gearbox + can length (measure)
MOTOR_SHAFT_D = 4.0
MOTOR_SHAFT_FLAT = 3.5   # D-cut: centre to flat = 3.5 - 2.0 = 1.5mm
MOTOR_SHAFT_LEN = 10.0   # (measure)
MOTOR_HOLE_PITCH = 17.0  # M3 holes on the gearbox face (measure)
MOTOR_HOLE_D = 3.2
GEARBOX_CLAMP_LEN = 18.0 # clamped length of the 25mm gearbox body (gearbox is ~19-24mm long, measure)

# caster
CASTER_H = 25.0          # ball caster height floor -> mounting face (measure)
CASTER_D = 32.0

# gear: module 1, 20 deg
MODULE = 1.0
PRESSURE_ANGLE = 20.0
PINION_TEETH = 20
GEAR_FACE = 12.0

# gripper
OPEN_INNER = 400.0
CLOSED_INNER = 130.0
STROKE = (OPEN_INNER - CLOSED_INNER) / 2  # per side = 135
ARM_LEN = 170.0          # frame front edge -> arm tip
ARM_T = 6.0
PAD_Z_MIN = 30.0
PAD_Z_MAX = 120.0
PAD_T = 5.0              # pad incl. rubber
DECK_T = 5.0             # gripper deck under the guide blocks
RACK_Z = 140.0           # rack / pinion bottom face; above the arm plates so racks pass over the far arm

# ultrasonic HC-SR04
SONAR_Z = 45.0
SONAR_SPACING = 200.0
SONAR_BOARD = (45.0, 20.0)
SONAR_CAN_D = 16.0
SONAR_CAN_PITCH = 26.0
SONAR_CAN_H = 12.0

# electronics bay and battery, both on the rear member and the mid cross member
MID_CROSS_X = -18.0      # centre of the extra cross member, just clear of the wheels
BOARD = (90.0, 75.0)     # perfboard
TRAY_Y = (-66.0, 66.0)   # electronics tray |y| range (left of the battery, right of the e-stop)
ELECTRONICS_H = 45.0     # space kept above the tray for the board, drivers, buck module and wiring
BATTERY = (106.0, 35.0, 26.0)  # LiPo 3S 2200mAh class pack, L x W x H (measure)
BATTERY_Y = -97.5        # battery centre line, right of the tray

# rules / targets
BOX = 150.0
ENVELOPE_MAX = 500.0
