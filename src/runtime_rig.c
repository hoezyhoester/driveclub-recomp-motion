/* Motion rig telemetry: the player's car as UDP packets for a motion simulator, through SpaceMonkey (its DriveClub
 * provider), which hands them on in Codemasters' custom UDP format to Sim Racing Studio, SimTools, FlyPT Mover...
 *
 * Where it comes from - the game's own simulation, not the camera. The game passes its physics steps only to the
 * force feedback of a Logitech-type wheel; without a wheel, the car's world velocity and angular velocity (the
 * camera's vehicle target, below) give the same, by their change: no per-wheel suspension then (flag 8 off).
 * With a wheel:
 *   - Every physics step the force feedback inputs hook (runtime_pad.c) has the player's car state. Per wheel, in
 *     the game's order right front, left front, right rear, left rear (the car's x axis points left): +0xa50 load,
 *     +0xa60 lateral force, +0xa70 longitudinal force, in N. The block the game passed on: +0x18 speed (m/s).
 *       sway   (lateral g)  = the tyres' lateral forces over the car's weight - what an accelerometer in the car
 *                             reads, banking included;
 *       heave  (vertical g) = the tyres' loads over the weight, less 1 g: crests, dips, bumps and kerbs;
 *       surge  (longit. g)  = the change of speed: braking, accelerating, drag, and hitting things head on;
 *       per wheel           = each tyre's load against its own recent average, as suspension travel: a kerb on the
 *                             left is felt on the left, road texture on the shakers.
 *     The weight is learnt from the loads while the car is slow (no downforce yet).
 *   - The camera's vehicle target (runtime_rig_camera.inc) carries the car's linear and angular velocity in the
 *     world: the yaw rate (spins, slides), the road's slope (pitch), and impacts with walls and other cars, which the
 *     tyres do not feel. The target that is the player's car is the one whose speed matches the physics' speed.
 *     Which world axis is up and which way the yaw and the slope count are learnt while driving, from the tyres
 *     (a right-hander pulls the car right; uphill the tyres push more than the car speeds up); the log says when
 *     (lines starting "Rig:").
 *
 * Settings: in data\user\input_config\default.ini (motion_* lines), or the environment for a test run:
 *   motion = on|off            on by default (BB_MOTION=0 turns it off)
 *   motion_host = 127.0.0.1    the PC SpaceMonkey runs on (BB_MOTION_HOST)
 *   motion_port = 20790        (BB_MOTION_PORT)
 *   motion_flip_vertical = off on when heave and slope come out upside down (BB_MOTION_FLIP_VERTICAL)
 *
 * The packet (SpaceMonkey's BurnoutRevengeAPI.cs reads it), little-endian, 192 bytes:
 *    0 char[4] "XRTM"      4 u16 version 2       6 u16 flags: 1 in a race, 2 inputs, 4 dynamics, 8 suspension
 *    8 u32 sequence       12 u32 game: 2 = DriveClub
 *   16 f64 time (s)       24 f64[3] position (0: not sent)  48 f64[3] forward  72 f64[3] up (from yaw and pitch)
 *   96 f32 speed (m/s)   100 f32 throttle, brake, steering (-1..1), rpm, gear (-1: not known)  120 f32 clutch
 *  124 f32 0
 *  128 f32 acceleration right, up, forward (m/s^2; up is 0 at rest)
 *  140 f32 yaw rate (rad/s, + turning right)   144 f32 pitch (rad, + nose up)   148 f32 roll (rad, 0: not known)
 *  152 f32 yaw (rad)      156 f32 impact (g, 0..3, fading over 0.15 s)
 *  160 f32[4] suspension travel FL FR RL RR (mm, + compressed)   176 f32[4] wheel load over its average (1 at rest)
 *  192 f32[4] slip ratio FL FR RL RR (0 rolling, + spinning, - locking)   208 f32[4] slip angle (rad)   (flag 16)
 * The tyres come from the physics steps with a wheel, or from the car's state found through the camera's target
 * without one (runtime_rig_scan.inc); RPM and gear once that search has found them (-1 until then).
 */
#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif
#include <math.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "runtime_rig.h"

#define RIG_PACKET 224
#define RIG_G 9.81f
#define RIG_TARGETS 8
#define RIG_PI 3.14159265f

static uint64_t rig_now_us(void) {
    struct timespec t; clock_gettime(CLOCK_MONOTONIC,&t);
    return (uint64_t)t.tv_sec*1000000u+(uint64_t)t.tv_nsec/1000u;
}
static float rig_clamp(float v, float lo, float hi) { return v<lo ? lo : v>hi ? hi : v; }
static bool rig_finite3(const float *v) { return isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]); }
static void rig_v_cross(const float *a, const float *b, float *o) {
    o[0]=a[1]*b[2]-a[2]*b[1]; o[1]=a[2]*b[0]-a[0]*b[2]; o[2]=a[0]*b[1]-a[1]*b[0];
}
static float rig_v_dot(const float *a, const float *b) { return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; }
static float rig_v_len(const float *a) { return sqrtf(rig_v_dot(a,a)); }
#include "runtime_rig_scan.inc"

/* ---- settings -------------------------------------------------------------------------------------------------- */
static struct { bool enabled; char host[64]; int port; bool decided; bool flip_up; } rig_cfg={true,"127.0.0.1",20790,false,false};
static pthread_mutex_t rig_m=PTHREAD_MUTEX_INITIALIZER;

static bool rig_on(const char *v) {   /* anything but 0, off, false or no, in any case */
    char l[16]; size_t i=0;
    for (; v[i] && i<sizeof l-1; ++i) l[i]=(char)(v[i]>='A' && v[i]<='Z' ? v[i]+32 : v[i]);
    l[i]=0;
    return strcmp(l,"0") && strcmp(l,"off") && strcmp(l,"false") && strcmp(l,"no");
}

/* motion_* lines of default.ini (runtime_input.inc); the environment, read last, wins. */
void runtime_rig_config(const char *key, const char *value) {
    pthread_mutex_lock(&rig_m);
    if (!strcmp(key,"motion")) rig_cfg.enabled=rig_on(value);
    else if (!strcmp(key,"motion_host")) snprintf(rig_cfg.host,sizeof rig_cfg.host,"%s",value);
    else if (!strcmp(key,"motion_port")) { const int p=atoi(value); if (p>0 && p<65536) rig_cfg.port=p; }
    else if (!strcmp(key,"motion_flip_vertical")) rig_cfg.flip_up=rig_on(value);
    pthread_mutex_unlock(&rig_m);
}
static void rig_decide(void) {
    if (rig_cfg.decided) return;
    rig_cfg.decided=true;
    const char *e;
    if ((e=getenv("BB_MOTION")) && *e) rig_cfg.enabled=rig_on(e);
    if ((e=getenv("BB_MOTION_HOST")) && *e) snprintf(rig_cfg.host,sizeof rig_cfg.host,"%s",e);
    if ((e=getenv("BB_MOTION_PORT")) && *e) { const int p=atoi(e); if (p>0 && p<65536) rig_cfg.port=p; }
    if ((e=getenv("BB_MOTION_FLIP_VERTICAL")) && *e) rig_cfg.flip_up=rig_on(e);
    if (rig_cfg.enabled) printf("Rig: motion telemetry to %s:%d\n",rig_cfg.host,rig_cfg.port);
    else printf("Rig: motion telemetry off\n");
}

/* ---- the socket ----------------------------------------------------------------------------------------------- */
#ifdef _WIN32
static SOCKET rig_socket=INVALID_SOCKET;
#else
static int rig_socket=-1;
#endif
static struct sockaddr_in rig_to;
static bool rig_opened, rig_ok;

static bool rig_open(void) {
    if (rig_opened) return rig_ok;
    rig_opened=true;
#ifdef _WIN32
    WSADATA wsa; WSAStartup(MAKEWORD(2,2),&wsa);
    rig_socket=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
    if (rig_socket==INVALID_SOCKET) { printf("Rig: no UDP socket; nothing is sent\n"); return false; }
    u_long nonblocking=1; ioctlsocket(rig_socket,FIONBIO,&nonblocking);
#else
    rig_socket=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
    if (rig_socket<0) { printf("Rig: no UDP socket; nothing is sent\n"); return false; }
#endif
    memset(&rig_to,0,sizeof rig_to);
    rig_to.sin_family=AF_INET;
    rig_to.sin_port=htons((uint16_t)rig_cfg.port);
    if (inet_pton(AF_INET,rig_cfg.host,&rig_to.sin_addr)!=1) {
        printf("Rig: motion_host '%s' is not an IPv4 address; nothing is sent\n",rig_cfg.host); return false;
    }
    rig_ok=true;
    return true;
}
static void rig_send(const uint8_t *p, size_t n) {
    if (!rig_open()) return;
#ifdef _WIN32
    sendto(rig_socket,(const char *)p,(int)n,0,(const struct sockaddr *)&rig_to,sizeof rig_to);
#else
    sendto(rig_socket,p,n,MSG_DONTWAIT,(const struct sockaddr *)&rig_to,sizeof rig_to);
#endif
}

/* ---- the camera's vehicle targets ----------------------------------------------------------------------------- */
static struct {
    const void *target; uint64_t us;
    float lin[3], ang[3], acc[3];   /* world velocity, angular velocity, and the change of velocity (m/s^2) */
    bool has_acc;
    float dt;                       /* the time the last change of velocity was over */
    unsigned hits, last_hits; uint64_t hits_since;   /* updates in the last second: the camera's own car has most */
} rig_targets[RIG_TARGETS];
/* What the player presses, read before rig_m is taken: the pad code holds its own lock while it reads the motion
 * settings (runtime_rig_config), so the two are never taken the other way round. */
typedef struct { bool inputs, steered; float throttle, brake, clutch, steering; } RigInputs;
static RigInputs rig_inputs(void) {
    RigInputs in={false,false,-1,-1,-1,0};
    in.inputs=runtime_pad_pedals(&in.throttle,&in.brake,&in.clutch)>0;
    in.steered=runtime_pad_steering(&in.steering)>0;
    if (!in.steered) in.steering=0.0f;
    return in;
}
static void rig_camera_step(int slot, uint64_t now, const RigInputs *in);
static void rig_decide(void);

void runtime_rig_camera_target(const void *target) {
    if (!target || !rig_cfg.enabled) return;
    float lin[3], ang[3];
    memcpy(lin,(const char *)target+0x120,12);
    memcpy(ang,(const char *)target+0x130,12);
    if (!rig_finite3(lin) || !rig_finite3(ang) || fabsf(lin[0])+fabsf(lin[1])+fabsf(lin[2])>500.0f) return;
    const uint64_t now=rig_now_us();
    const RigInputs in=rig_inputs();
    pthread_mutex_lock(&rig_m);
    int slot=-1, oldest=0;
    for (int i=0;i<RIG_TARGETS;++i) {
        if (rig_targets[i].target==target) { slot=i; break; }
        if (rig_targets[i].us<rig_targets[oldest].us) oldest=i;
    }
    if (slot<0) { slot=oldest; memset(&rig_targets[slot],0,sizeof rig_targets[slot]); rig_targets[slot].target=target; rig_targets[slot].hits_since=now; }
    if (now-rig_targets[slot].hits_since>=1000000) {
        rig_targets[slot].last_hits=rig_targets[slot].hits; rig_targets[slot].hits=0; rig_targets[slot].hits_since=now;
    }
    ++rig_targets[slot].hits;
    const float dt=rig_targets[slot].us ? (float)(now-rig_targets[slot].us)*1e-6f : 0.0f;
    /* The physics steps at its own rate and the camera at the frame rate: the same velocity on two frames is one
     * sample, and the change of velocity is over the time since it last changed (a still car: every 0.1 s). */
    const bool same=!memcmp(lin,rig_targets[slot].lin,12) && !memcmp(ang,rig_targets[slot].ang,12);
    bool fresh=false;
    if (dt==0.0f) fresh=true;
    else if (dt>0.002f && (!same || dt>=0.1f)) {
        if (dt<0.25f) {
            for (int k=0;k<3;++k) rig_targets[slot].acc[k]=(lin[k]-rig_targets[slot].lin[k])/dt;
            rig_targets[slot].has_acc=true; rig_targets[slot].dt=dt;
        } else rig_targets[slot].has_acc=false;
        fresh=true;
    }
    if (fresh) {
        memcpy(rig_targets[slot].lin,lin,12); memcpy(rig_targets[slot].ang,ang,12);
        rig_targets[slot].us=now;
        rig_decide();
        rig_camera_step(slot,now,&in);
    }
    pthread_mutex_unlock(&rig_m);
}

/* ---- the car ------------------------------------------------------------------------------------------------- */
static struct {
    bool car;                 /* a car under the player's control at the last step */
    uint64_t us, idle_us;
    uint32_t seq;
    float weight;             /* the car's weight (N), learnt slow */
    float wheel_avg[4];       /* each tyre's recent load (N) */
    float speed;              /* the last step's speed */
    float surge;              /* the change of speed, smoothed (m/s^2) */
    float yaw, pitch;
    /* learnt while driving: */
    int up_axis;              /* the world's up axis (-1 until known) */
    double axis_sum[3]; int axis_samples;
    double yaw_cov; int yaw_samples; float yaw_sign;
    double lon_cov; int lon_samples; float lon_sign;
    double slope_sum, resid_sum, slope_resid_sum, slope_sq_sum; int slope_samples; float up_sign;
    float impact;             /* decaying, g */
    uint64_t report_us;
    uint64_t sent_us;         /* the last packet of a car, for the idle thread */
    uint64_t ffb_us;          /* the last physics step from the force feedback hook (a wheel) */
} rig={.up_axis=-1,.yaw_sign=1.0f,.up_sign=1.0f,.lon_sign=1.0f};

/* Without the physics steps (no steering wheel: the game only passes its physics to the Logitech force feedback),
 * the car's motion from the camera's vehicle target: its world velocity and angular velocity. */
static struct {
    int lead;                 /* the target that is the player's car */
    uint64_t us, report_us;
    float f[3];               /* the way the car goes, flat */
    float a[3];               /* the change of velocity, smoothed (m/s^2) */
    float yaw, pitch, impact;
    int up_axis; double axis_sum[3]; int axis_samples;
    double right_cov; int right_samples; float right_sign;   /* which side is right: the way the player steers */
    double yaw_cov; int yaw_samples; float yaw_sign;
    bool told;
    /* the car's own object and its simulation state, found from the target (the idle thread) */
    uintptr_t target, vehicle, state; float target_lin[3]; uint64_t search_us; float throttle;
    char vehicle_how[96], state_how[48]; int state_bad;
} cam={.lead=-1,.f={0,0,1},.up_axis=-1,.right_sign=1.0f,.yaw_sign=1.0f};

static void rig_put_f(uint8_t *p, size_t at, float v) { memcpy(p+at,&v,4); }
static void rig_put_d(uint8_t *p, size_t at, double v) { memcpy(p+at,&v,8); }

static void rig_header(uint8_t *p, uint16_t flags, double t) {
    memcpy(p,"XRTM",4);
    const uint16_t version=2; memcpy(p+4,&version,2); memcpy(p+6,&flags,2);
    memcpy(p+8,&rig.seq,4); ++rig.seq;
    const uint32_t game=2; memcpy(p+12,&game,4);
    rig_put_d(p,16,t);
}

/* Ten times a second: when no car has been sent for a quarter of a second (menus, loading, pause), "not in a
 * race", so the motion software eases the rig to rest. */
static void rig_send_idle(uint64_t now) {
    if (now-rig.idle_us<100000) return;
    rig.idle_us=now;
    uint8_t p[RIG_PACKET]={0};
    rig_header(p,0,(double)now*1e-6);
    rig_put_d(p,48+16,1.0); rig_put_d(p,72+8,1.0);   /* forward z, up y */
    for (size_t at=100;at<=116;at+=4) rig_put_f(p,at,-1.0f);
    rig_send(p,sizeof p);
}
static void *rig_idle_thread(void *arg) {
    (void)arg;
    uint64_t last=rig_now_us();
    for (;;) {
        struct timespec t={0,50000000L}; nanosleep(&t,NULL);
        pthread_mutex_lock(&rig_m);
        const uint64_t now=rig_now_us();
        if (now-rig.sent_us>250000) {
            if (cam.lead>=0 && cam.told) { printf("Rig: no car moving; the rig rests\n"); cam.told=false; }
            rig_send_idle(now);
        }
        /* what the search needs, copied out: it reads the game's memory without rig_m held */
        const bool moving=now-rig.sent_us<250000;
        const uintptr_t target=cam.target, vehicle=cam.vehicle, state=cam.state;
        float lin[3]; memcpy(lin,cam.target_lin,12);
        const float throttle=cam.throttle, speed=rig_v_len(lin);
        const bool search=moving && !vehicle && target && speed>3.0f && now-cam.search_us>1000000;
        if (search) cam.search_us=now;
        pthread_mutex_unlock(&rig_m);
        const float dt=(float)(now-last)*1e-6f; last=now;

        if (search) {
            char how[96]="", state_how[48]="";
            const uintptr_t v=rig_find_vehicle(target,lin,how,sizeof how);
            const uintptr_t s=v ? rig_find_state(v,state_how,sizeof state_how) : 0;
            static unsigned tries;
            if (v || ++tries%30==1) {
                if (v) printf("Rig: the car's object found (%s)%s%s\n",how,s ? "; its tyres at " : "; its tyres not found",s ? state_how : "");
                else printf("Rig: the car's object not found from the camera yet\n");
            }
            if (v) {
                pthread_mutex_lock(&rig_m);
                cam.vehicle=v; cam.state=s; cam.told=false; cam.state_bad=0;
                snprintf(cam.vehicle_how,sizeof cam.vehicle_how,"%s",how);
                snprintf(cam.state_how,sizeof cam.state_how,"%s",state_how);
                pthread_mutex_unlock(&rig_m);
            }
        }
        if (moving && vehicle && dt>0.0f && dt<0.5f) scan_sample(vehicle,state,speed,throttle,dt);
    }
    return NULL;
}
/* From scePadInit: the settings are read, and the idle packets start. */
void runtime_rig_start(void) {
    pthread_mutex_lock(&rig_m);
    rig_decide();
    static bool started;
    if (rig_cfg.enabled && !started) {
        started=true;
        pthread_t t;
        if (pthread_create(&t,NULL,rig_idle_thread,NULL)==0) pthread_detach(t);
    }
    pthread_mutex_unlock(&rig_m);
}

void runtime_rig_no_car(void) {
    pthread_mutex_lock(&rig_m);
    rig_decide();
    if (rig_cfg.enabled) {
        const uint64_t now=rig_now_us();
        if (rig.car) { rig.car=false; printf("Rig: no car under the player's control\n"); }
        rig.us=0;
        rig_send_idle(now);
    }
    pthread_mutex_unlock(&rig_m);
}

/* The vehicle target that is the player's car: updated in the last 0.1 s, its speed the physics' speed. */
static int rig_match(float speed, uint64_t now) {
    int best=-1; float best_diff=1e9f;
    for (int i=0;i<RIG_TARGETS;++i) {
        if (!rig_targets[i].target || now-rig_targets[i].us>100000) continue;
        const float *v=rig_targets[i].lin;
        const float diff=fabsf(sqrtf(v[0]*v[0]+v[1]*v[1]+v[2]*v[2])-speed);
        if (diff<best_diff) { best_diff=diff; best=i; }
    }
    return best>=0 && best_diff<fmaxf(2.0f,0.1f*speed) ? best : -1;
}

static void rig_physics_locked(const float *st, const uint8_t *block, const RigInputs *in_p, uint64_t now);
void runtime_rig_physics_step(const float *st, const uint8_t *block) {
    const RigInputs in=rig_inputs();
    pthread_mutex_lock(&rig_m);
    rig_decide();
    if (rig_cfg.enabled && st && block) {
        const uint64_t now=rig_now_us();
        rig.ffb_us=now;
        rig_physics_locked(st,block,&in,now);
    }
    pthread_mutex_unlock(&rig_m);
}
/* One step of the car from its simulation state st (the force feedback hook's, or the one found from the camera's
 * target): under rig_m. */
static void rig_physics_locked(const float *st, const uint8_t *block, const RigInputs *in_p, uint64_t now) {
    const RigInputs in=*in_p;
    float speed; memcpy(&speed,block+0x18,4);
    const float *load=st+0xa50/4, *lat=st+0xa60/4, *lon=st+0xa70/4;
    float sum_load=0, sum_lat=0, sum_lon=0;
    for (int i=0;i<4;++i) {
        if (!isfinite(load[i]) || !isfinite(lat[i]) || !isfinite(lon[i]) || fabsf(load[i])>1e6f) return;
        sum_load+=load[i]; sum_lat+=lat[i]; sum_lon+=lon[i];
    }
    if (!isfinite(speed)) speed=0.0f;
    if (!rig.car) {
        rig.car=true; rig.us=0; rig.surge=0; rig.impact=0; rig.yaw=0; rig.pitch=0;
        for (int i=0;i<4;++i) rig.wheel_avg[i]=load[i];
        printf("Rig: the player's car%s, %.0f N on its tyres\n",rig.ffb_us==now ? " (from the wheel's force feedback)" : " (found from the camera)",sum_load);
    }
    float dt=rig.us ? (float)(now-rig.us)*1e-6f : 0.0f;
    const bool stepped=dt>0.0005f && dt<0.1f;
    rig.us=now;

    /* the weight, while slow enough that downforce does not count */
    if (speed<25.0f && sum_load>2000.0f && sum_load<80000.0f) {
        if (rig.weight<=0.0f) rig.weight=sum_load;
        else if (stepped) rig.weight+=(sum_load-rig.weight)*fminf(1.0f,dt/3.0f);
    }
    const float weight=rig.weight>0.0f ? rig.weight : (sum_load>2000.0f ? sum_load : 15000.0f);

    /* sway and heave from the tyres; surge from the change of speed */
    const float right=-sum_lat/weight*RIG_G;            /* m/s^2, + to the right (the car's x points left) */
    const float up=(sum_load/weight-1.0f)*RIG_G;        /* 0 at rest */
    float dv=0.0f;
    if (stepped) {
        dv=(speed-rig.speed)/dt;
        if (fabsf(dv)<400.0f) rig.surge+=(dv-rig.surge)*fminf(1.0f,dt/0.03f);
    }
    rig.speed=speed;
    /* which way the tyres' longitudinal forces count: with the change of speed */
    if (stepped && speed>5.0f && fabsf(dv)>1.5f && fabsf(dv)<30.0f && rig.lon_samples<1000) {
        rig.lon_cov+=(double)sum_lon*dv;
        if (++rig.lon_samples==300 || rig.lon_samples==1000) rig.lon_sign=rig.lon_cov<0.0 ? -1.0f : 1.0f;
    }

    /* the camera's vehicle target that is this car */
    const int t=rig_match(speed,now);
    float yaw_rate=speed>3.0f ? right/speed : 0.0f;    /* without the target: the turn the sway asks for */
    float slope=0.0f;
    if (t>=0) {
        const float *lin=rig_targets[t].lin, *ang=rig_targets[t].ang;
        /* up: the axis the car hardly moves along */
        if (rig.up_axis<0 && speed>10.0f) {
            for (int k=0;k<3;++k) rig.axis_sum[k]+=fabsf(lin[k]);
            if (++rig.axis_samples>=300) {
                int a=0; for (int k=1;k<3;++k) if (rig.axis_sum[k]<rig.axis_sum[a]) a=k;
                rig.up_axis=a;
                printf("Rig: the world's up axis is %c\n","xyz"[a]);
            }
        }
        if (rig.up_axis>=0) {
            const int a=rig.up_axis;
            /* the yaw: a right-hander's yaw goes with a pull to the right */
            const float raw=ang[a];
            if (speed>8.0f && fabsf(right)>2.0f && rig.yaw_samples<2000) {
                rig.yaw_cov+=(double)raw*right;
                if (++rig.yaw_samples==200 || rig.yaw_samples==2000) {
                    const float s=rig.yaw_cov<0.0 ? -1.0f : 1.0f;
                    if (s!=rig.yaw_sign || rig.yaw_samples==200) printf("Rig: yaw counts %s\n",s>0 ? "as the game's" : "against the game's");
                    rig.yaw_sign=s;
                }
            }
            yaw_rate=rig.yaw_sign*raw;
            /* the slope: uphill the tyres push more than the car speeds up */
            float flat=0.0f; for (int k=0;k<3;++k) if (k!=a) flat+=lin[k]*lin[k];
            flat=sqrtf(flat);
            if (flat>5.0f) {
                const float s_raw=atanf(rig_clamp(lin[a]/flat,-0.6f,0.6f));
                if (stepped && rig.lon_samples>=300 && rig.slope_samples<5000) {
                    const double resid=(double)rig.lon_sign*sum_lon/weight*RIG_G-rig.surge;   /* g sin(slope) + drag */
                    rig.slope_sum+=s_raw; rig.resid_sum+=resid; rig.slope_resid_sum+=s_raw*resid; rig.slope_sq_sum+=(double)s_raw*s_raw;
                    if (++rig.slope_samples==1000 || rig.slope_samples==5000) {
                        const double n=rig.slope_samples;
                        const double cov=rig.slope_resid_sum/n-(rig.slope_sum/n)*(rig.resid_sum/n);
                        const double var=rig.slope_sq_sum/n-(rig.slope_sum/n)*(rig.slope_sum/n);
                        if (var>1e-5) {
                            rig.up_sign=cov<0.0 ? -1.0f : 1.0f;
                            printf("Rig: uphill is %s%c\n",rig.up_sign>0 ? "+" : "-","xyz"[a]);
                        }
                    }
                }
                slope=rig.up_sign*s_raw;
            }
        }
        /* an impact: the car's change of velocity past what its tyres can do */
        if (rig_targets[t].has_acc) {
            const float *acc=rig_targets[t].acc;
            const float kin=sqrtf(acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2])/RIG_G;
            const float tyres=sqrtf(sum_lat*sum_lat+sum_lon*sum_lon)/weight+fabsf(sum_load/weight-1.0f);
            const float hit=kin-tyres-1.0f;
            /* past 60 g it is the car put back on the track, not a crash */
            if (kin<60.0f && hit>rig.impact) rig.impact=fminf(hit,3.0f);
        }
    }
    if (stepped) {
        rig.impact*=expf(-dt/0.15f);
        rig.pitch+=(slope-rig.pitch)*fminf(1.0f,dt/0.15f);
        rig.yaw+=yaw_rate*dt;
        if (rig.yaw>RIG_PI) rig.yaw-=2.0f*RIG_PI;
        if (rig.yaw<-RIG_PI) rig.yaw+=2.0f*RIG_PI;
    }

    /* each tyre against its own recent load, as suspension travel */
    float travel[4], ratio[4];
    for (int i=0;i<4;++i) {
        if (stepped) rig.wheel_avg[i]+=(load[i]-rig.wheel_avg[i])*fminf(1.0f,dt/2.0f);
        const float avg=fmaxf(rig.wheel_avg[i],200.0f);
        ratio[i]=load[i]/avg;
        travel[i]=rig_clamp((ratio[i]-1.0f)*40.0f,-80.0f,80.0f);
    }

    if (now-rig.report_us>30000000) {
        rig.report_us=now;
        printf("Rig: %.0f km/h, weight %.0f N, sway %.2f g, heave %.2f g, surge %.2f g, yaw rate %.2f, pitch %.1f deg, camera %s\n",
               speed*3.6f,weight,right/RIG_G,up/RIG_G,rig.surge/RIG_G,yaw_rate,rig.pitch*57.3f,t>=0 ? "matched" : "not matched");
    }

    uint8_t p[RIG_PACKET]={0};
    const float throttle=in.throttle, brake=in.brake, clutch=in.clutch, steering=in.steering;
    const bool inputs=in.inputs;
    rig_header(p,(uint16_t)(1|(inputs ? 2 : 0)|4|8|16),(double)now*1e-6);
    /* forward and up from the yaw and pitch, for display */
    const float cy=cosf(rig.yaw), sy=sinf(rig.yaw), cp=cosf(rig.pitch), sp=sinf(rig.pitch);
    rig_put_d(p,48,sy*cp); rig_put_d(p,56,sp); rig_put_d(p,64,cy*cp);
    rig_put_d(p,72,-sy*sp); rig_put_d(p,80,cp); rig_put_d(p,88,-cy*sp);
    rig_put_f(p,96,speed);
    rig_put_f(p,100,inputs ? throttle : -1.0f); rig_put_f(p,104,inputs ? brake : -1.0f);
    float rpm, gear; scan_live(&rpm,&gear);
    rig_put_f(p,108,steering); rig_put_f(p,112,rpm); rig_put_f(p,116,gear);
    rig_put_f(p,120,inputs ? clutch : -1.0f);
    rig_put_f(p,128,right); rig_put_f(p,132,up); rig_put_f(p,136,rig.surge);
    rig_put_f(p,140,yaw_rate); rig_put_f(p,144,rig.pitch); rig_put_f(p,148,0.0f); rig_put_f(p,152,rig.yaw);
    rig_put_f(p,156,rig.impact);
    /* the game's right front, left front, right rear, left rear as front left, front right, rear left, rear right */
    static const int order[4]={1,0,3,2};
    const float *slip_ratio=st+0xb30/4, *slip_angle=st+0xb50/4;
    for (int i=0;i<4;++i) {
        rig_put_f(p,160+4*i,travel[order[i]]); rig_put_f(p,176+4*i,ratio[order[i]]);
        const float sr=slip_ratio[order[i]], sa=slip_angle[order[i]];
        rig_put_f(p,192+4*i,isfinite(sr) ? rig_clamp(sr,-1.0f,5.0f) : 0.0f);
        rig_put_f(p,208+4*i,isfinite(sa) ? rig_clamp(sa,-1.6f,1.6f) : 0.0f);
    }
    rig_send(p,sizeof p);
    rig.sent_us=now;
}

/* ---- without the physics: the camera's vehicle target ------------------------------------------------------ */

static void rig_camera_step(int slot, uint64_t now, const RigInputs *in) {
    if (!rig_cfg.enabled) return;
    /* the physics steps, when the game passes them (a Logitech wheel), are better */
    if (rig.ffb_us && now-rig.ffb_us<250000) return;
    /* the player's car: the target updated most often; another only once this one stops */
    if (cam.lead!=slot) {
        const bool lead_alive=cam.lead>=0 && rig_targets[cam.lead].target && now-rig_targets[cam.lead].us<250000;
        if (lead_alive && rig_targets[slot].last_hits<=rig_targets[cam.lead].last_hits) return;
        cam.lead=slot; cam.us=0;
        printf("Rig: following the camera's car (target %p); without a Logitech wheel the motion comes from its velocity\n",
               rig_targets[slot].target);
    }
    const float *v=rig_targets[slot].lin, *w=rig_targets[slot].ang;
    const float speed=rig_v_len(v);
    /* for the idle thread's search */
    cam.target=(uintptr_t)rig_targets[slot].target; memcpy(cam.target_lin,v,12); cam.throttle=in->inputs ? in->throttle : 0.0f;
    /* the car's own state, once found: its tyres, as with a wheel */
    if (cam.state) {
        static float st[0xc00/4];
        if (rig_read(cam.state,st,sizeof st) && rig_tyres_plausible(st)) {
            cam.state_bad=0;
            uint8_t block[0x30]={0}; memcpy(block+0x18,&speed,4);
            if (!cam.told) { cam.told=true; printf("Rig: motion from the car's tyres (found without a wheel)\n"); }
            rig_physics_locked(st,block,in,now);
            return;
        }
        if (++cam.state_bad>30) { printf("Rig: the car's state no longer reads right; back to its velocity\n"); cam.state=0; cam.vehicle=0; cam.told=false; }
    }
    if (!cam.told) { cam.told=true; printf("Rig: motion from the car's velocity\n"); }
    const float dt=cam.us ? fminf(0.1f,fmaxf(0.001f,(float)(now-cam.us)*1e-6f)) : 0.016f;
    cam.us=now;

    /* up: the axis the car hardly moves along (y until known) */
    if (cam.up_axis<0 && speed>10.0f) {
        for (int k=0;k<3;++k) cam.axis_sum[k]+=fabsf(v[k]);
        if (++cam.axis_samples>=300) {
            int a=0; for (int k=1;k<3;++k) if (cam.axis_sum[k]<cam.axis_sum[a]) a=k;
            cam.up_axis=a;
            printf("Rig: the world's up axis is %s%c\n",rig_cfg.flip_up ? "-" : "+","xyz"[a]);
        }
    }
    const int ax=cam.up_axis>=0 ? cam.up_axis : 1;
    float U[3]={0,0,0}; U[ax]=rig_cfg.flip_up ? -1.0f : 1.0f;

    /* the way the car goes, flat; kept while it is (nearly) still */
    float flat[3]={v[0],v[1],v[2]}; flat[ax]=0.0f;
    const float flat_speed=rig_v_len(flat);
    if (flat_speed>1.5f) for (int k=0;k<3;++k) cam.f[k]=flat[k]/flat_speed;
    float R0[3]; rig_v_cross(cam.f,U,R0);

    /* the change of velocity, smoothed; a jump past 60 g is the car put back on the track */
    float raw[3]={0,0,0};
    if (rig_targets[slot].has_acc) memcpy(raw,rig_targets[slot].acc,12);
    const float raw_g=rig_v_len(raw)/RIG_G;
    if (raw_g>60.0f) { memset(raw,0,12); memset(cam.a,0,12); }
    const float k_a=fminf(1.0f,dt/0.05f);
    for (int k=0;k<3;++k) cam.a[k]+=(raw[k]-cam.a[k])*k_a;
    if (raw_g<60.0f && raw_g>3.0f && raw_g-3.0f>cam.impact) cam.impact=fminf(raw_g-3.0f,3.0f);
    cam.impact*=expf(-dt/0.15f);

    /* which side is right: a car steered right is pushed right */
    const float steering=in->steering;
    const bool steered=in->steered;
    const float side=rig_v_dot(cam.a,R0);
    if (steered && speed>8.0f && fabsf(steering)>0.25f && fabsf(side)>2.0f && cam.right_samples<2000) {
        cam.right_cov+=(double)steering*side;
        if (++cam.right_samples==150 || cam.right_samples==2000) {
            const float s=cam.right_cov<0.0 ? -1.0f : 1.0f;
            if (s!=cam.right_sign || cam.right_samples==150) printf("Rig: right is %s the world's (from the steering)\n",s>0 ? "as" : "against");
            cam.right_sign=s;
        }
    }
    float R[3]; for (int k=0;k<3;++k) R[k]=R0[k]*cam.right_sign;
    const float right=rig_v_dot(cam.a,R), up=rig_v_dot(cam.a,U), fwd=rig_v_dot(cam.a,cam.f);

    /* the yaw rate, + turning right: with the steering, as the side */
    const float w_up=rig_v_dot(w,U);
    if (steered && speed>8.0f && fabsf(steering)>0.25f && fabsf(w_up)>0.05f && cam.yaw_samples<2000) {
        cam.yaw_cov+=(double)w_up*steering;
        if (++cam.yaw_samples==150 || cam.yaw_samples==2000) cam.yaw_sign=cam.yaw_cov<0.0 ? -1.0f : 1.0f;
    }
    const float yaw_rate=cam.yaw_sign*w_up;
    cam.yaw+=yaw_rate*dt;
    if (cam.yaw>RIG_PI) cam.yaw-=2.0f*RIG_PI;
    if (cam.yaw<-RIG_PI) cam.yaw+=2.0f*RIG_PI;

    /* the slope along the way it goes */
    const float slope=flat_speed>5.0f ? atanf(rig_clamp(rig_v_dot(v,U)/flat_speed,-0.6f,0.6f)) : 0.0f;
    cam.pitch+=(slope-cam.pitch)*fminf(1.0f,dt/0.15f);

    if (now-cam.report_us>30000000) {
        cam.report_us=now;
        printf("Rig: %.0f km/h, sway %.2f g, heave %.2f g, surge %.2f g, yaw rate %.2f, pitch %.1f deg (from the velocity)\n",
               speed*3.6f,right/RIG_G,up/RIG_G,fwd/RIG_G,yaw_rate,cam.pitch*57.3f);
    }

    uint8_t p[RIG_PACKET]={0};
    const float throttle=in->throttle, brake=in->brake, clutch=in->clutch;
    const bool inputs=in->inputs;
    rig_header(p,(uint16_t)(1|(inputs ? 2 : 0)|4),(double)now*1e-6);
    const float cy=cosf(cam.yaw), sy=sinf(cam.yaw), cp=cosf(cam.pitch), sp=sinf(cam.pitch);
    rig_put_d(p,48,sy*cp); rig_put_d(p,56,sp); rig_put_d(p,64,cy*cp);
    rig_put_d(p,72,-sy*sp); rig_put_d(p,80,cp); rig_put_d(p,88,-cy*sp);
    rig_put_f(p,96,speed);
    rig_put_f(p,100,inputs ? throttle : -1.0f); rig_put_f(p,104,inputs ? brake : -1.0f);
    float rpm, gear; scan_live(&rpm,&gear);
    rig_put_f(p,108,steered ? steering : 0.0f); rig_put_f(p,112,rpm); rig_put_f(p,116,gear);
    rig_put_f(p,120,inputs ? clutch : -1.0f);
    rig_put_f(p,128,right); rig_put_f(p,132,up); rig_put_f(p,136,fwd);
    rig_put_f(p,140,yaw_rate); rig_put_f(p,144,cam.pitch); rig_put_f(p,148,0.0f); rig_put_f(p,152,cam.yaw);
    rig_put_f(p,156,cam.impact);
    for (int i=0;i<4;++i) rig_put_f(p,176+4*i,1.0f);
    rig_send(p,sizeof p);
    rig.sent_us=now;
}
