# Passive hardware convention checks

`sudo ./scripts/run_conventions.sh NEW.csv` records 60 seconds using the production
USB/decoder/input-reader path. Always wheel-zero/joint Exit, never controller
algorithm, Enter, set-zero or calibration write. Outputs go to CSV only after
Stop/Close. Invalid/stale fields remain cached; interpret values only alongside
freshness/motor masks. No automated physical correctness PASS is issued.

Current configuration, not yet physical proof:
- CAN3 left: wheel 0x201, B 0x010, D 0x013; CAN2 right: wheel 0x201,
  B 0x011, D 0x012.
- Joint positions/velocities use DM MIT ranges, radians/rad/s. Link installation
  offsets B=0 and D=pi; leg left/right coordinate transforms are in chassis_params.
- Wheel raw encoder and motor RPM are captured before feedback direction/reduction
  correction. Configured reduction=15.7647058824, radius=0.058 m;
  feedback signs left=+1 right=-1. These require actual hardware verification.
- HI91 angles remain protocol pitch/roll/yaw in degrees; rates are X->pitch,
  Y->roll, Z->yaw multiplied by pi/180. Acceleration remains protocol g.

One labelled action per run avoids ambiguous signs. First pitch test: maintain
a supported, unpowered-actuation chassis; remain still for ~10 s, gently lower
the front of the chassis without rolling the wheels for ~10 s, hold, restore,
then remain still. Stop with Ctrl-C if the chassis cannot be held securely.
Record approximate action times and report the actual direction. Compare angle
change with corresponding rate sign/integral; do not assume a positive pitch
means nose-up before the physical/body-axis convention is agreed.

Next separate runs: lateral tilt, individual wheel forward rolling (only with
mechanical support suitable for manual motion), and each joint/link pose. Do not
force joints or alter zero points. Mapping requires observing which physical
part moves; absolute zero/reduction needs a known measured pose/displacement,
not simply a plausible telemetry value. Command torque signs are not validated
by zero-output/manual-motion testing and require a later limited-output test.
