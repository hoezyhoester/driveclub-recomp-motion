/* The motion rig (runtime_rig.c) reads the car's world velocity and angular velocity from the camera's vehicle
 * target: CameraTarget::Update (guest +0xccd600) leaves, for a target of kind 1 (a vehicle), the vehicle's linear
 * velocity at +0x120 and its angular velocity at +0x130 (runtime_camera_motion.inc samples the same two). With
 * BB_CAMERA_PATCH=1 that update is already hooked there and passes each target on; otherwise it is hooked here. */
#include "runtime_rig.h"
static CameraTargetUpdate rig_camera_update_original;
static void rig_camera_observe(void *self) {
    unsigned kind;
    if (!camera_pose_read((uintptr_t)self+8,&kind,4) || kind!=1) return;
    runtime_rig_camera_target(self);
}
static ABI void rig_camera_target_update(void *self,unsigned frame,float dt) {
    rig_camera_update_original(self,frame,dt);
    rig_camera_observe(self);
}
static void rig_camera_install(void) {
    if (camera_target_update_original) { puts("Rig: camera vehicle targets observed through the camera patch"); return; }
    static const unsigned char entry[]={0x55,0x48,0x89,0xe5,0x41,0x57,0x41,0x56,0x53,0x48,0x81,0xec,0x88,0,0,0};
    rig_camera_update_original=(CameraTargetUpdate)camera_rate_hook(0xccd600,entry,sizeof entry,(uintptr_t)rig_camera_target_update);
    printf("Rig: camera vehicle targets %s\n",rig_camera_update_original ? "observed" : "unavailable (the game's code differs); yaw and slope come from the tyres alone");
}
