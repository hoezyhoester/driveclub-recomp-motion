/* Motion rig telemetry: the player's car as UDP packets for a motion simulator, through SpaceMonkey (its DriveClub
 * provider), which hands them on in Codemasters' custom UDP format to Sim Racing Studio, SimTools, FlyPT Mover...
 *
 * Where it comes from - the game's own simulation, not the camera:
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

#define RIG_PACKET 192
#define RIG_G 9.81f
#define RIG_TARGETS 8
#define RIG_PI 3.14159265f

static uint64_t rig_now_us(void) {
    struct timespec t; clock_gettime(CLOCK_MONOTONIC,&t);
    return (uint64_t)t.tv_sec*1000000u+(uint64_t)t.tv_nsec/1000u;
}
static float rig_clamp(float v, float lo, float hi) { return v<lo ? lo : v>hi ? hi : v; }
static bool rig_finite3(const float *v) { return isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]); }

/* ---- settings -------------------------------------------------------------------------------------------------- */
static struct { bool enabled; char host[64]; int port; bool decided; } rig_cfg={true,"127.0.0.1",20790,false};
static pthread_mutex_t rig_m=PTHREAD_MUTEX_INITIALIZER;

static bool rig_on(const char *v) { return strcmp(v,"0") && strcmp(v,"off") && strcmp(v,"false") && strcmp(v,"no"); }

/* motion_* lines of default.ini (runtime_input.inc); the environment, read last, wins. */
void runtime_rig_config(const char *key, const char *value) {
    pthread_mutex_lock(&rig_m);
    if (!strcmp(key,"motion")) rig_cfg.enabled=rig_on(value);
    else if (!strcmp(key,"motion_host")) snprintf(rig_cfg.host,sizeof rig_cfg.host,"%s",value);
    else if (!strcmp(key,"motion_port")) { const int p=atoi(value); if (p>0 && p<65536) rig_cfg.port=p; }
    pthread_mutex_unlock(&rig_m);
}
static void rig_decide(void) {
    if (rig_cfg.decided) return;
    rig_cfg.decided=true;
    const char *e;
    if ((e=getenv("BB_MOTION")) && *e) rig_cfg.enabled=rig_on(e);
    if ((e=getenv("BB_MOTION_HOST")) && *e) snprintf(rig_cfg.host,sizeof rig_cfg.host,"%s",e);
    if ((e=getenv("BB_MOTION_PORT")) && *e) { const int p=atoi(e); if (p>0 && p<65536) rig_cfg.port=p; }
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
} rig_targets[RIG_TARGETS];

void runtime_rig_camera_target(const void *target) {
    if (!target || !rig_cfg.enabled) return;
    float lin[3], ang[3];
    memcpy(lin,(const char *)target+0x120,12);
    memcpy(ang,(const char *)target+0x130,12);
    if (!rig_finite3(lin) || !rig_finite3(ang) || fabsf(lin[0])+fabsf(lin[1])+fabsf(lin[2])>500.0f) return;
    const uint64_t now=rig_now_us();
    pthread_mutex_lock(&rig_m);
    int slot=-1, oldest=0;
    for (int i=0;i<RIG_TARGETS;++i) {
        if (rig_targets[i].target==target) { slot=i; break; }
        if (rig_targets[i].us<rig_targets[oldest].us) oldest=i;
    }
    if (slot<0) { slot=oldest; memset(&rig_targets[slot],0,sizeof rig_targets[slot]); rig_targets[slot].target=target; }
    const float dt=rig_targets[slot].us ? (float)(now-rig_targets[slot].us)*1e-6f : 0.0f;
    if (dt>0.002f && dt<0.1f) {
        for (int k=0;k<3;++k) rig_targets[slot].acc[k]=(lin[k]-rig_targets[slot].lin[k])/dt;
        rig_targets[slot].has_acc=true;
    } else if (dt>=0.1f) rig_targets[slot].has_acc=false;
    if (dt==0.0f || dt>0.002f) {   /* the same frame twice: keep the first */
        memcpy(rig_targets[slot].lin,lin,12); memcpy(rig_targets[slot].ang,ang,12);
        rig_targets[slot].us=now;
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
} rig={.up_axis=-1,.yaw_sign=1.0f,.up_sign=1.0f,.lon_sign=1.0f};

static void rig_put_f(uint8_t *p, size_t at, float v) { memcpy(p+at,&v,4); }
static void rig_put_d(uint8_t *p, size_t at, double v) { memcpy(p+at,&v,8); }

static void rig_header(uint8_t *p, uint16_t flags, double t) {
    memcpy(p,"XRTM",4);
    const uint16_t version=2; memcpy(p+4,&version,2); memcpy(p+6,&flags,2);
    memcpy(p+8,&rig.seq,4); ++rig.seq;
    const uint32_t game=2; memcpy(p+12,&game,4);
    rig_put_d(p,16,t);
}

void runtime_rig_no_car(void) {
    pthread_mutex_lock(&rig_m);
    rig_decide();
    if (rig_cfg.enabled) {
        const uint64_t now=rig_now_us();
        if (rig.car) { rig.car=false; printf("Rig: no car under the player's control\n"); }
        /* "not in a race", ten times a second: the motion software eases the rig to rest */
        if (now-rig.idle_us>=100000) {
            rig.idle_us=now;
            uint8_t p[RIG_PACKET]={0};
            rig_header(p,0,(double)now*1e-6);
            rig_put_d(p,48+16,1.0); rig_put_d(p,72+8,1.0);   /* forward z, up y */
            for (size_t at=100;at<=116;at+=4) rig_put_f(p,at,-1.0f);
            rig_send(p,sizeof p);
        }
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

void runtime_rig_physics_step(const float *st, const uint8_t *block) {
    pthread_mutex_lock(&rig_m);
    rig_decide();
    if (!rig_cfg.enabled || !st || !block) { pthread_mutex_unlock(&rig_m); return; }
    const uint64_t now=rig_now_us();
    float speed; memcpy(&speed,block+0x18,4);
    const float *load=st+0xa50/4, *lat=st+0xa60/4, *lon=st+0xa70/4;
    float sum_load=0, sum_lat=0, sum_lon=0;
    for (int i=0;i<4;++i) {
        if (!isfinite(load[i]) || !isfinite(lat[i]) || !isfinite(lon[i]) || fabsf(load[i])>1e6f) {
            pthread_mutex_unlock(&rig_m); return;
        }
        sum_load+=load[i]; sum_lat+=lat[i]; sum_lon+=lon[i];
    }
    if (!isfinite(speed)) speed=0.0f;
    if (!rig.car) {
        rig.car=true; rig.us=0; rig.surge=0; rig.impact=0; rig.yaw=0; rig.pitch=0;
        for (int i=0;i<4;++i) rig.wheel_avg[i]=load[i];
        printf("Rig: the player's car (state %p), %.0f N on its tyres\n",(const void *)st,sum_load);
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
    float throttle=-1, brake=-1, clutch=-1, steering=-1;
    const bool inputs=runtime_pad_pedals(&throttle,&brake,&clutch)>0;
    if (!runtime_pad_steering(&steering)) steering=0.0f;
    rig_header(p,(uint16_t)(1|(inputs ? 2 : 0)|4|8),(double)now*1e-6);
    /* forward and up from the yaw and pitch, for display */
    const float cy=cosf(rig.yaw), sy=sinf(rig.yaw), cp=cosf(rig.pitch), sp=sinf(rig.pitch);
    rig_put_d(p,48,sy*cp); rig_put_d(p,56,sp); rig_put_d(p,64,cy*cp);
    rig_put_d(p,72,-sy*sp); rig_put_d(p,80,cp); rig_put_d(p,88,-cy*sp);
    rig_put_f(p,96,speed);
    rig_put_f(p,100,inputs ? throttle : -1.0f); rig_put_f(p,104,inputs ? brake : -1.0f);
    rig_put_f(p,108,steering); rig_put_f(p,112,-1.0f); rig_put_f(p,116,-1.0f);
    rig_put_f(p,120,inputs ? clutch : -1.0f);
    rig_put_f(p,128,right); rig_put_f(p,132,up); rig_put_f(p,136,rig.surge);
    rig_put_f(p,140,yaw_rate); rig_put_f(p,144,rig.pitch); rig_put_f(p,148,0.0f); rig_put_f(p,152,rig.yaw);
    rig_put_f(p,156,rig.impact);
    /* the game's right front, left front, right rear, left rear as front left, front right, rear left, rear right */
    static const int order[4]={1,0,3,2};
    for (int i=0;i<4;++i) { rig_put_f(p,160+4*i,travel[order[i]]); rig_put_f(p,176+4*i,ratio[order[i]]); }
    rig_send(p,sizeof p);
    pthread_mutex_unlock(&rig_m);
}
