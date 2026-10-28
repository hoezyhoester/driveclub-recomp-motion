# Motion rig telemetry

The game sends what the player's car does, every physics step, as UDP packets for a motion simulator. The packets
go to SpaceMonkey (<https://github.com/PHARTGAMES/SpaceMonkey>, its **DriveClub** button), which hands them on in
Codemasters' custom UDP format to Sim Racing Studio, SimTools, SimFeedback, FlyPT Mover and SimHub.

It is on by default and sends to this PC (`127.0.0.1`, port 20790), where SpaceMonkey listens. Nothing listening
costs nothing.

## Settings

In `data\user\input_config\default.ini`:

```
motion = on              ; off to stop sending
motion_host = 127.0.0.1  ; the PC SpaceMonkey runs on
motion_port = 20790
```

For a test run the environment wins: `BB_MOTION=0`, `BB_MOTION_HOST`, `BB_MOTION_PORT`.

## What is sent, and where it comes from

All from the game's own simulation of the car (`src/runtime_rig.c`), not from the camera:

| | From |
|---|---|
| Sway (lateral g) | the tyres' lateral forces over the car's weight, as an accelerometer in the car reads it |
| Heave (vertical g) | the tyres' loads over the weight: crests, dips, bumps, kerbs, landings |
| Surge (longitudinal g) | the change of the car's speed: braking, accelerating, drag, head-on hits |
| Suspension, per wheel | each tyre's load against its own recent average: a kerb on the left is felt on the left; road texture for shakers |
| Yaw rate | the car's angular velocity (the camera's vehicle target): slides and spins |
| Pitch | the road's slope along the way the car goes |
| Impact | the car's change of velocity past what its tyres can do: walls and other cars |
| Pedals and steering | what the game was given, wheel or pad |

Roll (the road's camber) is not sent yet: banking shows in the sway instead, as on a real car.

The car's weight, which world axis is up and which way yaw and slope count are learnt in the first minute of
driving. The log says when, in lines starting `Rig:`, and every 30 seconds what it sends.

## The packet

Little-endian, 192 bytes ("XRTM" version 2; version 1 is Xerenge's Burnout Revenge packet, which SpaceMonkey reads
too):

| Offset | Type | |
|---|---|---|
| 0 | char[4] | `XRTM` |
| 4 | u16 | version, 2 |
| 6 | u16 | flags: 1 in a race, 2 inputs, 4 accelerations, 8 suspension |
| 8 | u32 | sequence |
| 12 | u32 | game, 2 = DriveClub |
| 16 | f64 | time, seconds |
| 24 | f64[3] | position (0: not sent) |
| 48 | f64[3] | forward (from yaw and pitch) |
| 72 | f64[3] | up |
| 96 | f32 | speed, m/s |
| 100 | f32[5] | throttle, brake (0..1), steering (-1..1), rpm, gear (-1: not known) |
| 120 | f32 | clutch |
| 128 | f32[3] | acceleration right, up, forward (m/s², up 0 at rest) |
| 140 | f32 | yaw rate (rad/s, + turning right) |
| 144 | f32 | pitch (rad, + nose up) |
| 148 | f32 | roll (0: not known) |
| 152 | f32 | yaw (rad) |
| 156 | f32 | impact (g, 0..3) |
| 160 | f32[4] | suspension travel front left, front right, rear left, rear right (mm, + compressed) |
| 176 | f32[4] | each wheel's load over its average (1 at rest), same order |
