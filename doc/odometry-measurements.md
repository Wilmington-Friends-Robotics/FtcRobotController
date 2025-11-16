# Odometry Measurements

Robot: odometry testing chassis

## Chassis
- Length: 17 in
- Width: 10.38 in

## Longitudinal Tracking Wheel (forward/back)
- From back edge: 4.75 in
- Longitudinal offset to robot center: 3.75 in backward
- From left edge: 3.2 in
- Lateral offset to robot center: 1.9 in leftward
- Forward = positive delta, backward = negative delta

## Lateral Tracking Wheel (strafe)
- From back edge: 3.875 in
- Longitudinal offset to robot center: 4.625 in backward
- From right edge: 4.75 in
- Lateral offset to robot center: 0.44 in right
- Right = positive delta, left = negative delta

## Both Wheels:
- Diameter: 32mm
- CPR: 2000

## Control Hub Devices:
Motors:
- Bus 0: goBILDA 5203 series "front_right"
- Bus 1: goBILDA 5203 series "front_left"
- Bus 2: goBILDA 5203 series "back_right"
- Bus 3: goBILDA 5203 series "back_left"
IC2:
- Bus 0: goBILDA Pinpoint Odometry Computer "odometry"
