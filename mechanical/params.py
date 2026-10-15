"""All dimensions (mm). Coordinates: origin on the floor under the frame centre,
+X forward, +Y left, +Z up. Values marked (measure) must be checked on the real part."""

# frame (20mm aluminium extrusion)
FRAME_W = 300.0          # along Y
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
RAIL_SECTION = 10.0      # square aluminium bar for the rack slides
RAIL_Z = 135.0           # rail centre height (above the frame, below box top)

# ultrasonic HC-SR04
SONAR_Z = 45.0
SONAR_SPACING = 200.0
SONAR_BOARD = (45.0, 20.0)
SONAR_CAN_D = 16.0
SONAR_CAN_PITCH = 26.0
SONAR_CAN_H = 12.0

# perfboard
BOARD = (100.0, 80.0)

# rules / targets
BOX = 150.0
ENVELOPE_MAX = 500.0
