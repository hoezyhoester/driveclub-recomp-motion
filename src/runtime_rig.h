#ifndef RUNTIME_RIG_H
#define RUNTIME_RIG_H
/* Motion rig telemetry (runtime_rig.c): the player's car, from the game's own simulation, as UDP packets for
 * SpaceMonkey and through it Sim Racing Studio, SimTools, FlyPT Mover and the like. */
#include <stdint.h>

/* One physics step of the player's car (runtime_pad.c's force feedback inputs hook, the game's thread): the car's
 * state (tyre loads and forces at +0xa50..) and the block the game passed on (+0x18 speed). */
void runtime_rig_physics_step(const float *state, const uint8_t *block);
/* The physics step ran with no car under the player's control. */
void runtime_rig_no_car(void);
/* A camera target was updated (runtime_rig_camera.inc): for kind 1, a vehicle, +0x120 its linear and +0x130 its
 * angular velocity in the world. */
void runtime_rig_camera_target(const void *target);

/* The in-game display (HUD) updated for this car (its subject): where the rev counter and gear come from. */
void runtime_rig_hud_subject(const void *subject);
/* From scePadInit: reads the settings and starts the idle packets. */
void runtime_rig_start(void);
/* A motion line of default.ini (motion, motion_host, motion_port). */
void runtime_rig_config(const char *key, const char *value);

/* runtime_pad.c: what the player presses, for the packet. Pedals 0..1, steering -1 (left) .. 1 (right). */
int runtime_pad_pedals(float *throttle, float *brake, float *clutch);
int runtime_pad_steering(float *steering);
#endif
