/* The motion rig's search of the game's memory (included by runtime_rig.c).
 *
 * 1. The car: the camera's vehicle target holds a copy of the car's velocity (+0x120); the car's own object, which
 *    it was copied from, is found among the objects the target points to, by that velocity. From the car,
 *    +0x68 -> +0x10 is its simulation state (runtime_replay_recorded_motion.inc), whose tyres (+0xa50 loads,
 *    +0xa60 lateral, +0xa70 longitudinal forces, +0xb30 slip ratio, +0xb50 slip angle) are what the force feedback
 *    reads with a wheel. So the tyres come through with a gamepad too, when this finds them; the log says.
 * 2. RPM and gear: not known in this build. While the car is driven, fields of the car, its state and the objects
 *    they point to are watched: an engine speed rises with the car's speed and drops at each upshift while the
 *    car keeps going; a gear is a small whole number that steps up at those drops. The best candidates go to the
 *    log ("Rig: RPM candidate", "Rig: gear candidate") and, when one stands out, it is used straight away.
 *
 * Every read goes through ReadProcessMemory (Windows): a stale pointer reads nothing rather than faulting. */

#ifdef _WIN32
#include <windows.h>
static bool rig_read(uintptr_t address, void *out, size_t size) {
    SIZE_T got=0;
    return address>=0x10000 && address<0x7fffffffffffull &&
           ReadProcessMemory(GetCurrentProcess(),(const void *)address,out,size,&got) && got==size;
}
#else
/* tests: the same, through process_vm_readv */
#include <sys/uio.h>
static bool rig_read(uintptr_t address, void *out, size_t size) {
    if (address<0x10000 || address>=0x7fffffffffffull) return false;
    struct iovec local={out,size}, remote={(void *)address,size};
    return process_vm_readv(getpid(),&local,1,&remote,1,0)==(ssize_t)size;
}
#endif

static bool rig_pointer(uint64_t p) { return p>=0x10000 && p<0x7fffffffffffull && !(p&7); }

/* ---- 1. the car --------------------------------------------------------------------------------------------- */

/* Where velocity v (3 floats) lies in the 0x800 bytes at object; -1 if not there. */
static int rig_find_velocity(uintptr_t object, const float *v) {
    static float buf[0x800/4];
    if (!rig_read(object,buf,sizeof buf)) return -1;
    for (int i=0;i+2<(int)(sizeof buf/4);++i)
        if (buf[i]==v[0] && buf[i+1]==v[1] && buf[i+2]==v[2]) return i*4;
    return -1;
}

/* The car object the target's velocity came from: one of the objects it points to, or one those point to. */
static uintptr_t rig_find_vehicle(uintptr_t target, const float *v, char *how, size_t how_size) {
    uint64_t ptrs[0x400/8];
    if (!rig_read(target,ptrs,sizeof ptrs)) return 0;
    for (int i=0;i<(int)(sizeof ptrs/8);++i) {
        if (!rig_pointer(ptrs[i]) || ptrs[i]==target) continue;
        const int at=rig_find_velocity(ptrs[i],v);
        if (at>=0) { snprintf(how,how_size,"target+0x%x, velocity at +0x%x",i*8,at); return (uintptr_t)ptrs[i]; }
    }
    for (int i=0;i<(int)(sizeof ptrs/8);++i) {
        if (!rig_pointer(ptrs[i]) || ptrs[i]==target) continue;
        uint64_t inner[0x200/8];
        if (!rig_read(ptrs[i],inner,sizeof inner)) continue;
        for (int j=0;j<(int)(sizeof inner/8);++j) {
            if (!rig_pointer(inner[j]) || inner[j]==target || inner[j]==ptrs[i]) continue;
            const int at=rig_find_velocity(inner[j],v);
            if (at>=0) {
                snprintf(how,how_size,"target+0x%x -> +0x%x, velocity at +0x%x",i*8,j*8,at);
                return (uintptr_t)inner[j];
            }
        }
    }
    return 0;
}

/* Tyre loads that make sense: four, none past 50 kN, together what a car weighs (with downforce). */
static bool rig_tyres_plausible(const float *st) {
    const float *load=st+0xa50/4, *lat=st+0xa60/4, *lon=st+0xa70/4;
    float sum=0;
    for (int i=0;i<4;++i) {
        if (!isfinite(load[i]) || !isfinite(lat[i]) || !isfinite(lon[i])) return false;
        if (load[i]<-100.0f || load[i]>50000.0f || fabsf(lat[i])>1e5f || fabsf(lon[i])>1e5f) return false;
        sum+=load[i];
    }
    return sum>1500.0f && sum<120000.0f;
}

/* Objects that could be the car's simulation state: vehicle +0x68 -> +0x10 (the replay code's path), and every
 * object the car points to (its first 0x1000 bytes), or one level further, whose tyres look right. Which one is
 * live is told by rig_tyre_test below: a copy or a template reads right but never moves. */
#define RIG_STATE_CANDIDATES 16
static uintptr_t rig_rejected[64]; static int rig_n_rejected;
static bool rig_is_rejected(uintptr_t a) { for (int i=0;i<rig_n_rejected;++i) if (rig_rejected[i]==a) return true; return false; }
static void rig_reject(uintptr_t a) { if (rig_n_rejected<64 && !rig_is_rejected(a)) rig_rejected[rig_n_rejected++]=a; }

typedef struct { uintptr_t at; char how[40]; } RigStateCandidate;
static int rig_add_state(RigStateCandidate *c, int n, uintptr_t at, const char *how) {
    if (n>=RIG_STATE_CANDIDATES || rig_is_rejected(at)) return n;
    for (int i=0;i<n;++i) if (c[i].at==at) return n;
    static float st[0xc00/4];
    if (!rig_read(at,st,sizeof st) || !rig_tyres_plausible(st)) return n;
    c[n].at=at; snprintf(c[n].how,sizeof c[n].how,"%s",how);
    return n+1;
}
/* Breadth first from the car, three levels deep (the first 0x1000 bytes of the car, 0x400 of what it points to,
 * 0x200 below that), at most 4000 objects: the ones whose tyres look right. */
static int rig_find_states(uintptr_t vehicle, RigStateCandidate *c) {
    int n=0;
    uint64_t a=0, b=0;
    if (rig_read(vehicle+0x68,&a,8) && rig_pointer(a) && rig_read(a+0x10,&b,8) && rig_pointer(b))
        n=rig_add_state(c,n,(uintptr_t)b,"vehicle+0x68 -> +0x10");
    enum { MAX=4000 };
    static struct { uintptr_t at; int depth; char how[40]; } q[MAX];
    static uintptr_t seen[8192];
    memset(seen,0,sizeof seen);
    int head=0, tail=0;
    q[tail].at=vehicle; q[tail].depth=0; snprintf(q[tail].how,sizeof q[tail].how,"vehicle"); ++tail;
    seen[(vehicle>>3)&8191]=vehicle;
    static uint64_t ptrs[0x1000/8];
    while (head<tail && n<RIG_STATE_CANDIDATES) {
        const uintptr_t at=q[head].at; const int depth=q[head].depth; char how[40]; snprintf(how,sizeof how,"%s",q[head].how); ++head;
        if (depth>0) {
            float probe[12];
            if (rig_read(at+0xa50,probe,sizeof probe)) {
                float sum=0; bool ok=true;
                for (int i=0;i<4;++i) { if (!isfinite(probe[i]) || probe[i]<-100.0f || probe[i]>50000.0f) ok=false; sum+=probe[i]; }
                if (ok && sum>1500.0f && sum<120000.0f) n=rig_add_state(c,n,at,how);
            }
        }
        if (depth>=3) continue;
        const size_t span=depth==0 ? 0x1000 : depth==1 ? 0x400 : 0x200;
        if (!rig_read(at,ptrs,span)) continue;
        for (size_t i=0;i<span/8 && tail<MAX;++i) {
            const uint64_t p=ptrs[i];
            if (!rig_pointer(p)) continue;
            uintptr_t *slot=&seen[(p>>3)&8191];
            if (*slot==p) continue;
            *slot=p;
            q[tail].at=(uintptr_t)p; q[tail].depth=depth+1;
            if (depth==0) snprintf(q[tail].how,sizeof q[tail].how,"vehicle+0x%x",(unsigned)(i*8));
            else snprintf(q[tail].how,sizeof q[tail].how,"%.24s->+0x%x",how,(unsigned)(i*8));
            ++tail;
        }
    }
    return n;
}

/* Telling the live tyres from a copy: over three seconds of driving, the loads must move and the tyres' lateral
 * force must follow the push the car's own velocity shows (kinematic, + right). */
static struct {
    int n; RigStateCandidate c[RIG_STATE_CANDIDATES];
    int samples;
    float load_min[RIG_STATE_CANDIDATES], load_max[RIG_STATE_CANDIDATES];
    double sx[RIG_STATE_CANDIDATES], sy, sxx[RIG_STATE_CANDIDATES], syy, sxy[RIG_STATE_CANDIDATES];
} rig_test;

static void rig_tyre_test_start(const RigStateCandidate *c, int n) {
    memset(&rig_test,0,sizeof rig_test);
    rig_test.n=n; memcpy(rig_test.c,c,n*sizeof *c);
    for (int i=0;i<n;++i) { rig_test.load_min[i]=1e30f; rig_test.load_max[i]=-1e30f; }
}
/* One sample (the idle thread, 20 Hz, driving above 8 m/s). Returns the live state once decided, 0 before; all
 * candidates that fail are rejected. *done is set when the test ended (with or without a winner). */
static uintptr_t rig_tyre_test_sample(float kin_right, bool *done, char *how, size_t how_size) {
    *done=false;
    if (!rig_test.n) { *done=true; return 0; }
    rig_test.sy+=kin_right; rig_test.syy+=(double)kin_right*kin_right;
    for (int i=0;i<rig_test.n;++i) {
        float t[12];   /* loads +0xa50, lateral +0xa60, longitudinal +0xa70 */
        if (!rig_read(rig_test.c[i].at+0xa50,t,sizeof t)) { rig_test.load_min[i]=-1e30f; continue; }
        const float load=t[0]+t[1]+t[2]+t[3], lat=t[4]+t[5]+t[6]+t[7];
        if (load<rig_test.load_min[i]) rig_test.load_min[i]=load;
        if (load>rig_test.load_max[i]) rig_test.load_max[i]=load;
        const float x=load>500.0f ? -lat/load*RIG_G : 0.0f;
        rig_test.sx[i]+=x; rig_test.sxx[i]+=(double)x*x; rig_test.sxy[i]+=(double)x*kin_right;
    }
    if (++rig_test.samples<60) return 0;
    *done=true;
    const double n=rig_test.samples, my=rig_test.sy/n, vy=rig_test.syy/n-my*my;
    int best=-1; double best_r=0.8;
    for (int i=0;i<rig_test.n;++i) {
        const double mx=rig_test.sx[i]/n, vx=rig_test.sxx[i]/n-mx*mx, cov=rig_test.sxy[i]/n-mx*my;
        const double r=vx>1e-6 && vy>1e-6 ? cov/sqrt(vx*vy) : 0.0;
        const bool moving=rig_test.load_max[i]-rig_test.load_min[i]>100.0f;
        printf("Rig: tyres at %s: loads %.0f..%.0f N, lateral force against the car's turning r=%.2f%s\n",rig_test.c[i].how,
               rig_test.load_min[i],rig_test.load_max[i],r,moving && r>0.8 ? "" : " - not live");
        if (moving && r>best_r) { best=i; best_r=r; }
    }
    if (vy<0.5) {   /* too little cornering to tell: try again */
        printf("Rig: not enough cornering to test the tyres; trying again\n");
        rig_test.samples=0; rig_test.sy=rig_test.syy=0;
        for (int i=0;i<rig_test.n;++i) { rig_test.sx[i]=rig_test.sxx[i]=rig_test.sxy[i]=0; rig_test.load_min[i]=1e30f; rig_test.load_max[i]=-1e30f; }
        *done=false; return 0;
    }
    for (int i=0;i<rig_test.n;++i) if (i!=best) rig_reject(rig_test.c[i].at);
    if (best<0) return 0;
    snprintf(how,how_size,"%s",rig_test.c[best].how);
    return rig_test.c[best].at;
}

/* ---- 2. RPM and gear ------------------------------------------------------------------------------------- */

#define SCAN_OBJECTS 400
#define SCAN_BYTES 0x800
#define SCAN_FIELDS (SCAN_BYTES/4)
#define SCAN_HISTORY 4        /* samples back the drop is measured over (0.2 s at 20 Hz) */
#define SCAN_EVENTS 10

typedef struct {
    float min, max, hist[SCAN_HISTORY];
    float still_min, still_max;   /* while the car stands still: an engine idles there */
    uint16_t drops, n_events;
    float event_t[SCAN_EVENTS];   /* drop times (s) */
    bool in_range;
    /* against the car's speed: an engine's ratio to it holds within a gear and jumps between gears, so its
     * ratios gather into a few levels (a wheel's or a gearbox output's into one) */
    float ratio_prev, level[6];
    uint16_t ratio_n, ratio_stable, level_n[6];
} ScanFloat;
typedef struct {
    int32_t last; int16_t lo, hi; uint16_t ups, changes; bool ok;
    float up_t[SCAN_EVENTS]; uint16_t n_up;
} ScanInt;
typedef struct {
    uintptr_t address; char label[48];
    ScanFloat f[SCAN_FIELDS];
    ScanInt i[SCAN_FIELDS];
} ScanObject;

static struct {
    uintptr_t vehicle, state, hud;   /* what was scanned: a new car starts again */
    ScanObject *obj; int n_obj;
    unsigned samples, hist_pos; float drive_s;
    float speed_hist[SCAN_HISTORY];
    float t;                     /* driving time (s) */
    int reports; float next_report;
    /* what is used */
    uintptr_t rpm_at; float rpm_scale; uintptr_t gear_at; int gear_type;   /* 0 int32, 2 int32 less one */
    bool known;                  /* the places found on DriveClub 1.28: no search */
    float still_s;               /* how long the car has stood still */
    float frozen_s, frozen_val, frozen_speed;   /* a known place that stops moving while the car does: not live */
    float check_s, check_next;   /* the engine check lines: driving time, when the next is due */
    char rpm_label[64], gear_label[64];
    float rpm_max_seen;
} scan;

/* motion_rpm_scale in default.ini: a factor on the RPM found (for a unit the search took wrongly). */
static float rig_rpm_scale_cfg=1.0f;

/* The in-game display's car (its rev counter and gear come from it), from the display's update (runtime_rig.c). */
static volatile uintptr_t rig_hud_subject_at;

/* Where the engine's speed is, found by the search on a manual Mercedes (11 drops at 11 upshifts): the display's
 * car +0x1d8 -> +0x10, then +0x3d8, in rad/s (154..501 there: 1470..4780 rpm). +0x2ac in the same object drops at
 * upshifts too but reads down to 38 rad/s (370 rpm), below any idle: the gearbox's side, not the engine. */
static void rig_gear_reset(void);
#define EST_IDLE 850.0f
#define EST_RED 7000.0f
static struct {
    int gear; float rpm, top, hold; bool told;
    /* a manual gearbox on a pad: the gear follows the shift buttons; each gear's speed at the red line is learnt */
    bool manual, told_manual; unsigned up_seen, down_seen; int manual_gear; float red_v[8];
    bool reset;   /* a new race: first gear */
} est={1,EST_IDLE,0,0,false,false,false,0,0,1,{0},false};
static bool scan_known_display(uintptr_t hud) {
    rig_gear_reset();
    uint64_t a=0, b=0; float r;
    if (!hud || !rig_read(hud+0x1d8,&a,8) || !rig_pointer(a) || !rig_read(a+0x10,&b,8) || !rig_pointer(b)) return false;
    if (!rig_read(b+0x3d8,&r,4) || !isfinite(r) || r<0.0f || r>2000.0f) return false;
    scan.rpm_at=(uintptr_t)b+0x3d8; scan.rpm_scale=60.0f/(2.0f*3.14159265f);
    scan.gear_at=0;   /* the gear: from the engine's speed against the car's (rig_gear_from_ratio) */
    snprintf(scan.rpm_label,sizeof scan.rpm_label,"display car +0x1d8 -> +0x10 +0x3d8");
    scan.frozen_s=0; scan.frozen_val=r; scan.frozen_speed=-1.0f;
    printf("Rig: RPM from its known place (%s): now %.0f rpm; the gear is worked out from RPM against speed\n",
           scan.rpm_label,r*scan.rpm_scale);
    return true;
}

static void scan_reset(uintptr_t vehicle, uintptr_t state, uintptr_t hud) {
    if (!scan.obj) scan.obj=calloc(SCAN_OBJECTS,sizeof(ScanObject));
    if (!scan.obj) return;
    memset(scan.obj,0,SCAN_OBJECTS*sizeof(ScanObject));
    scan.vehicle=vehicle; scan.state=state; scan.hud=hud; scan.n_obj=0; scan.samples=0; scan.hist_pos=0; scan.t=0;
    scan.reports=0; scan.next_report=30.0f;
    { const char *e=getenv("BB_MOTION_SCAN_FIRST"); if (e && atof(e)>0) scan.next_report=(float)atof(e); }   /* tests */
    scan.rpm_at=0; scan.gear_at=0; scan.rpm_max_seen=0; scan.known=false;
    /* the objects: breadth first from the display's car and from the car found from the camera (and its state),
     * two levels deep (0x1000 bytes of each root, 0x400 below, 0x200 below that), the nearest SCAN_OBJECTS */
    const uintptr_t roots[3]={hud,vehicle,state};
    const char *names[3]={"display car","vehicle","state"};
    static struct { uintptr_t at; int depth; char label[48]; } q[4096];
    static uintptr_t seen[8192];
    memset(seen,0,sizeof seen);
    int head=0, tail=0;
    for (int r=0;r<3;++r) {
        if (!roots[r] || seen[(roots[r]>>3)&8191]==roots[r]) continue;
        seen[(roots[r]>>3)&8191]=roots[r];
        q[tail].at=roots[r]; q[tail].depth=0; snprintf(q[tail].label,sizeof q[tail].label,"%s",names[r]); ++tail;
    }
    static uint64_t ptrs[0x1000/8];
    static uint8_t probe[SCAN_BYTES];
    while (head<tail && scan.n_obj<SCAN_OBJECTS) {
        const uintptr_t at=q[head].at; const int depth=q[head].depth; char label[48]; snprintf(label,sizeof label,"%s",q[head].label); ++head;
        if (!rig_read(at,probe,sizeof probe)) continue;
        ScanObject *o=&scan.obj[scan.n_obj++]; o->address=at; snprintf(o->label,sizeof o->label,"%s",label);
        if (depth>=2) continue;
        const size_t span=depth==0 ? 0x1000 : 0x400;
        if (!rig_read(at,ptrs,span)) continue;
        for (size_t k=0;k<span/8 && tail<4096;++k) {
            const uint64_t p=ptrs[k];
            if (!rig_pointer(p) || seen[(p>>3)&8191]==p) continue;
            seen[(p>>3)&8191]=p;
            q[tail].at=(uintptr_t)p; q[tail].depth=depth+1;
            snprintf(q[tail].label,sizeof q[tail].label,"%.30s->+0x%x",label,(unsigned)(k*8));
            ++tail;
        }
    }
    for (int m=0;m<scan.n_obj;++m) for (int k=0;k<SCAN_FIELDS;++k) {
        scan.obj[m].f[k].in_range=true; scan.obj[m].f[k].min=1e30f; scan.obj[m].f[k].max=-1e30f;
        scan.obj[m].f[k].still_min=1e30f; scan.obj[m].f[k].still_max=-1e30f;
        scan.obj[m].i[k].ok=true; scan.obj[m].i[k].lo=32767; scan.obj[m].i[k].hi=-32768;
    }
    /* the display car's +0x1d8->+0x10 +0x3d8 read an engine on one drive and, on the next, 2200..2400 rpm all
     * the time - standing still and at the limiter alike: not the player's engine every time. Searched for now. */
    (void)scan_known_display;
    rig_gear_reset();
    est.top=0.0f; est.gear=1;
    if (!scan.known)
        printf("Rig: looking for the engine's RPM and gear in %d objects (%s the in-game display's car); drive a few minutes with gear changes\n",
               scan.n_obj,hud ? "with" : "without");
}

/* a field's value as RPM: rpm itself (300..16000), rad/s (30..1700), thousands, or a share of the dial */
static bool scan_rpm_range(float lo, float hi, float *scale) {
    if (lo>=300.0f && hi<=16000.0f && hi-lo>=1500.0f) { *scale=1.0f; return true; }
    if (lo>=30.0f && hi<=1700.0f && hi-lo>=150.0f) { *scale=60.0f/(2.0f*3.14159265f); return true; }
    /* a rev counter's own: thousands of rpm, or the needle's share of the dial (as 8000 rpm full scale) */
    if (lo>=0.3f && hi<=12.0f && hi-lo>=1.5f) { *scale=1000.0f; return true; }
    if (lo>=0.0f && hi<=1.2f && hi-lo>=0.3f) { *scale=8000.0f; return true; }
    return false;
}

/* one ratio sample: steady from the last one (2%) counts, and goes to its level (4%) */
static void scan_ratio(ScanFloat *f, float r) {
    const float prev=f->ratio_prev; f->ratio_prev=r;
    if (f->ratio_n<65535) ++f->ratio_n;
    if (!(prev>0.0f) || fabsf(r-prev)>0.02f*prev) return;
    if (f->ratio_stable<65535) ++f->ratio_stable;
    int best=-1; float bd=0.04f;
    for (int i=0;i<6;++i) if (f->level_n[i]) { const float d=fabsf(r-f->level[i])/f->level[i]; if (d<bd) { bd=d; best=i; } }
    if (best>=0) { if (f->level_n[best]<65535) ++f->level_n[best]; f->level[best]+=(r-f->level[best])/(float)(f->level_n[best]<64 ? f->level_n[best] : 64); return; }
    int at=-1; uint16_t least=65535;
    for (int i=0;i<6;++i) if (f->level_n[i]<least) { least=f->level_n[i]; at=i; }
    if (least<6) { f->level[at]=r; f->level_n[at]=1; }
}

/* how well a field moves like an engine against the speed: 0 when not; the levels that count, highest first */
static int scan_ratio_score(const ScanFloat *f, float *levels, int *n_levels) {
    *n_levels=0;
    if (f->ratio_n<200 || f->ratio_stable<0.6f*f->ratio_n) return 0;
    unsigned in_levels=0; float lv[6]; int n=0;
    for (int i=0;i<6;++i) if (f->level_n[i]>=0.06f*f->ratio_stable && f->level_n[i]>=15) { lv[n++]=f->level[i]; in_levels+=f->level_n[i]; }
    if (n<3 || in_levels<0.7f*f->ratio_stable) return 0;
    for (int a=0;a<n;++a) for (int b=a+1;b<n;++b) if (lv[b]>lv[a]) { const float t=lv[a]; lv[a]=lv[b]; lv[b]=t; }
    if (lv[0]<1.6f*lv[n-1]) return 0;
    for (int a=0;a<n;++a) levels[a]=lv[a];
    *n_levels=n;
    int score=n*100+(int)(100.0f*f->ratio_stable/f->ratio_n);
    /* a running engine never reads 0 while the car drives (0..8000 on 30 Oct was a switch, not an engine) */
    if (f->min<0.05f*f->max) return 0;
    if (f->still_max>=f->still_min) {   /* standing still: an engine idles, above zero and well below its top */
        const bool idles=f->still_min>0.03f*f->max && f->still_max<=0.5f*f->max;
        if (!idles) return 0;
        score+=200;
    }
    return score;
}

/* drops of field f matched by steps up of gear g, within 0.4 s */
static int scan_match(const ScanFloat *f, const ScanInt *g) {
    int m=0;
    for (int a=0;a<g->n_up && a<SCAN_EVENTS;++a)
        for (int b=0;b<f->n_events && b<SCAN_EVENTS;++b)
            if (fabsf(g->up_t[a]-f->event_t[b])<0.8f) { ++m; break; }
    return m;
}

static void scan_report(bool final) {
    typedef struct { int o, k; int score; float scale; int matched; } Cand;
    Cand rpm[10]={{0}}; int n_rpm=0;
    for (int m=0;m<scan.n_obj;++m) for (int k=0;k<SCAN_FIELDS;++k) {
        const ScanFloat *f=&scan.obj[m].f[k]; float scale;
        if (!f->in_range || f->drops<3 || !scan_rpm_range(f->min,f->max,&scale)) continue;
        bool dup=false;   /* a copy of one already listed: the same values (the same object, or a copy of it) */
        for (int x=0;x<n_rpm;++x) { const ScanFloat *o2=&scan.obj[rpm[x].o].f[rpm[x].k];
            if (o2->min==f->min && o2->max==f->max && o2->drops==f->drops) dup=true; }
        if (dup) continue;
        /* standing still, an engine idles: above zero and well below the top (when the car stood still at all) */
        int score=f->drops*10;
        if (f->still_max>=f->still_min) {
            const bool idles=f->still_min>0.0f && f->still_max<=0.45f*f->max;
            score+=idles ? 50 : -50;
        }
        Cand c={m,k,score,scale,0};
        if (n_rpm==10 && rpm[9].score>=c.score) continue;
        const int at=n_rpm<10 ? n_rpm++ : 9;
        rpm[at]=c;
        for (int x=at;x>0 && rpm[x].score>rpm[x-1].score;--x) { Cand t=rpm[x]; rpm[x]=rpm[x-1]; rpm[x-1]=t; }
    }
    Cand gear[6]={{0}}; int n_gear=0;
    for (int m=0;m<scan.n_obj;++m) for (int k=0;k<SCAN_FIELDS;++k) {
        const ScanInt *g=&scan.obj[m].i[k];
        if (!g->ok || g->ups<2 || g->hi-g->lo<2 || g->lo<-1 || g->hi>9) continue;
        /* a gear changes now and then: at most once every two seconds on average */
        if ((float)g->changes>0.5f*scan.t) continue;
        int best=0; for (int r=0;r<n_rpm;++r) { const int mm=scan_match(&scan.obj[rpm[r].o].f[rpm[r].k],g); if (mm>best) best=mm; }
        Cand c={m,k,best*1000+g->ups,1,best};
        if (n_gear==6 && gear[5].score>=c.score) continue;
        const int at=n_gear<6 ? n_gear++ : 5;
        gear[at]=c;
        for (int x=at;x>0 && gear[x].score>gear[x-1].score;--x) { Cand t=gear[x]; gear[x]=gear[x-1]; gear[x-1]=t; }
    }
    printf("Rig: after %.0f s of driving, %d RPM and %d gear candidates%s\n",scan.t,n_rpm,n_gear,final ? " (final)" : "");
    /* fields that move like an engine against the car's speed: a few steady levels, one a gear */
    typedef struct { int o, k, score, n; float lv[6], scale; } RCand;
    RCand rc[8]; memset(rc,0,sizeof rc); int n_rc=0;
    for (int m=0;m<scan.n_obj;++m) for (int k=0;k<SCAN_FIELDS;++k) {
        const ScanFloat *f=&scan.obj[m].f[k]; float scale; RCand c;
        if (!f->in_range) continue;
        c.score=scan_ratio_score(f,c.lv,&c.n);
        if (c.score<=0) continue;
        if (!scan_rpm_range(f->min,f->max,&scale)) {   /* a narrow range from little driving: by its top */
            if (f->max>=1000.0f && f->max<=16000.0f) scale=1.0f;
            else if (f->max>=100.0f && f->max<=1700.0f) scale=60.0f/(2.0f*3.14159265f);
            else continue;
        }
        bool dup=false;
        for (int x=0;x<n_rc;++x) { const ScanFloat *o2=&scan.obj[rc[x].o].f[rc[x].k]; if (o2->min==f->min && o2->max==f->max) dup=true; }
        if (dup) continue;
        c.o=m; c.k=k; c.scale=scale;
        /* full: only one better than the last replaces it (the last slot is never left unset) */
        if (n_rc==8 && rc[7].score>=c.score) continue;
        const int at=n_rc<8 ? n_rc++ : 7;
        rc[at]=c;
        for (int x=at;x>0 && rc[x].score>rc[x-1].score;--x) { RCand t=rc[x]; rc[x]=rc[x-1]; rc[x-1]=t; }
    }
    for (int r=0;r<n_rc;++r) {
        const ScanObject *o=&scan.obj[rc[r].o]; const ScanFloat *f=&o->f[rc[r].k];
        char line[512]; int n=snprintf(line,sizeof line,"Rig: engine candidate %d: %s +0x%x, %.0f..%.0f rpm, steady %.0f%%, ",r+1,o->label,rc[r].k*4,
                                       f->min*rc[r].scale,f->max*rc[r].scale,100.0f*f->ratio_stable/f->ratio_n);
        if (f->still_max>=f->still_min) n+=snprintf(line+n,sizeof line-n,"%.0f..%.0f rpm standing still, ",f->still_min*rc[r].scale,f->still_max*rc[r].scale);
        else n+=snprintf(line+n,sizeof line-n,"not seen standing still, ");
        n+=snprintf(line+n,sizeof line-n,"%d gears (rpm per km/h):",rc[r].n);
        for (int g=0;g<rc[r].n;++g) n+=snprintf(line+n,sizeof line-n," %.0f",rc[r].lv[g]*rc[r].scale/3.6f);
        printf("%s\n",line);
    }
    if (n_rc>0 && (n_rc==1 || rc[0].score-rc[1].score>=100 || scan.reports>=4)) {
        const ScanObject *o=&scan.obj[rc[0].o];
        const uintptr_t at=o->address+rc[0].k*4;
        if (at!=scan.rpm_at) {
            scan.rpm_at=at; scan.rpm_scale=rc[0].scale; scan.gear_at=0; rig_gear_reset();
            snprintf(scan.rpm_label,sizeof scan.rpm_label,"%s +0x%x",o->label,rc[0].k*4);
            printf("Rig: using RPM from %s; the gear is worked out from RPM against speed\n",scan.rpm_label);
        }
        return;
    }
    for (int r=0;r<n_rpm;++r) {
        const ScanObject *o=&scan.obj[rpm[r].o]; const ScanFloat *f=&o->f[rpm[r].k];
        char still[48]="not seen standing still";
        if (f->still_max>=f->still_min) snprintf(still,sizeof still,"%.3g..%.3g standing still",f->still_min,f->still_max);
        printf("Rig: RPM candidate %d: %s +0x%x, %.3g..%.3g%s, %u drops, %s\n",r+1,o->label,rpm[r].k*4,f->min,f->max,
               rpm[r].scale==1.0f ? " rpm" : rpm[r].scale==1000.0f ? " (thousands)" : rpm[r].scale==8000.0f ? " (share of the dial)" : " rad/s",(unsigned)f->drops,still);
    }
    for (int r=0;r<n_gear;++r) {
        const ScanObject *o=&scan.obj[gear[r].o]; const ScanInt *g=&o->i[gear[r].k];
        printf("Rig: gear candidate %d: %s +0x%x, %d..%d, %u steps up, %d at RPM drops\n",r+1,o->label,gear[r].k*4,
               (int)g->lo,(int)g->hi,(unsigned)g->ups,gear[r].matched);
    }
    /* the drops alone picked a gearbox's side and another car's engine before: listed, not used */
    if (false && n_rpm>0 && scan.obj[rpm[0].o].f[rpm[0].k].drops>=5 && (n_rpm==1 || rpm[0].score>=2*rpm[1].score || rpm[0].score-rpm[1].score>=60)) {
        const ScanObject *o=&scan.obj[rpm[0].o];
        scan.rpm_at=o->address+rpm[0].k*4; scan.rpm_scale=rpm[0].scale;
        snprintf(scan.rpm_label,sizeof scan.rpm_label,"%s +0x%x",o->label,rpm[0].k*4);
        printf("Rig: using RPM from %s\n",scan.rpm_label);
    }
    if (false && scan.rpm_at && n_gear>0 && gear[0].matched>=3) {
        const ScanObject *o=&scan.obj[gear[0].o];
        scan.gear_at=o->address+gear[0].k*4; scan.gear_type=0;
        snprintf(scan.gear_label,sizeof scan.gear_label,"%s +0x%x",o->label,gear[0].k*4);
        printf("Rig: using the gear from %s\n",scan.gear_label);
    }
}

/* For finding the engine's fields by eye: every two seconds for the first two minutes of a car, the speed, the
 * throttle and the object at the car's +0x540, around where its engine speed and gear were found, to the log. */
static void scan_check(uintptr_t vehicle, float speed, float throttle, float dt) {
    /* the display's data turned out a fixed copy (a manual Mercedes, 0..199 km/h: not one field moved); the
     * candidates' own lines say more now */
    if (getenv("BB_MOTION_ENGINE_CHECK")==NULL) { (void)vehicle; (void)speed; (void)throttle; (void)dt; return; }
    static uintptr_t last;
    const uintptr_t hud=rig_hud_subject_at;
    if (hud!=last) { last=hud; scan.check_s=0; scan.check_next=0; }
    (void)vehicle;
    scan.check_s+=dt;
    if (scan.check_s>120.0f || scan.check_s<scan.check_next) return;
    scan.check_next=scan.check_s+2.0f;
    uint64_t p=0; static float f[0x200/4];
    if (!hud) { printf("Rig: engine check: the in-game display's car is not known (no display update seen)\n"); return; }
    if (!rig_read(hud+0x10,&p,8) || !rig_pointer(p) || !rig_read(p,f,sizeof f)) {
        printf("Rig: engine check: the display's car data does not read\n"); return;
    }
    static char line[4096];
    int n=snprintf(line,sizeof line,"Rig: engine check %.0fs %3.0f km/h thr %.2f | display data floats",scan.check_s,speed*3.6f,throttle);
    for (int i=0;i<(int)(sizeof f/4) && n<(int)sizeof line-16;++i)
        n+=snprintf(line+n,sizeof line-n," %.4g",isfinite(f[i]) && fabsf(f[i])<1e7f ? f[i] : -1.0f);
    printf("%s\n",line);
    n=snprintf(line,sizeof line,"Rig: engine check %.0fs small ints",scan.check_s);
    const int32_t *iv=(const int32_t *)f;
    for (int i=0;i<(int)(sizeof f/4) && n<(int)sizeof line-24;++i)
        if (iv[i]>=-1 && iv[i]<=12) n+=snprintf(line+n,sizeof line-n," +0x%x=%d",i*4,(int)iv[i]);
    printf("%s\n",line);
}

/* One sample (the idle thread, 20 times a second) while the car is driven. */
static void scan_sample(uintptr_t vehicle, uintptr_t state, float speed, float throttle, float dt) {
    const uintptr_t hud=rig_hud_subject_at;
    if (vehicle!=scan.vehicle || state!=scan.state || hud!=scan.hud) scan_reset(vehicle,state,hud);
    if (!scan.obj || !scan.n_obj) return;
    scan_check(vehicle,speed,throttle,dt);
    if (scan.known) {   /* still there, and still moving with the car? */
        float r;
        if (rig_read(scan.rpm_at,&r,4) && isfinite(r) && r>=0.0f && r<2000.0f) {
            if (r!=scan.frozen_val) { scan.frozen_val=r; scan.frozen_s=0; scan.frozen_speed=speed; return; }
            if (scan.frozen_speed<0.0f) scan.frozen_speed=speed;
            scan.frozen_s+=dt;
            /* the same value to the bit for ten seconds while the speed changed by 10 m/s: not the engine */
            if (scan.frozen_s<10.0f || fabsf(speed-scan.frozen_speed)<10.0f) return;
            printf("Rig: the known RPM place stopped moving with the car; searching instead\n");
        } else printf("Rig: the known RPM place stopped reading right; searching instead\n");
        scan.known=false; scan.rpm_at=scan.gear_at=0;
    }
    const bool driving=speed>3.0f;
    if (!driving) {
        /* standing still: what each field reads then (an engine idles; most other things read 0 or stay) */
        if (speed<0.5f) {
            static uint8_t sbuf[SCAN_BYTES];
            for (int m=0;m<scan.n_obj;++m) {
                if (!rig_read(scan.obj[m].address,sbuf,sizeof sbuf)) continue;
                const float *fv=(const float *)sbuf;
                for (int k=0;k<SCAN_FIELDS;++k) {
                    ScanFloat *f=&scan.obj[m].f[k];
                    if (!f->in_range || !isfinite(fv[k])) continue;
                    if (fv[k]<f->still_min) f->still_min=fv[k];
                    if (fv[k]>f->still_max) f->still_max=fv[k];
                }
            }
        }
        return;
    }
    scan.t+=dt;
    const unsigned back=(scan.hist_pos+1)%SCAN_HISTORY;     /* the oldest sample */
    const float speed_then=scan.samples>=SCAN_HISTORY ? scan.speed_hist[back] : speed;
    const bool going=speed>=speed_then-0.3f && speed>5.0f && throttle>0.3f;
    static uint8_t buf[SCAN_BYTES];
    for (int m=0;m<scan.n_obj;++m) {
        ScanObject *o=&scan.obj[m];
        if (!rig_read(o->address,buf,sizeof buf)) continue;
        const float *fv=(const float *)buf; const int32_t *iv=(const int32_t *)buf;
        for (int k=0;k<SCAN_FIELDS;++k) {
            ScanFloat *f=&o->f[k];
            const float x=fv[k];
            if (f->in_range) {
                if (!isfinite(x) || x<0.0f || x>20000.0f) f->in_range=false;
                else {
                    if (x<f->min) f->min=x;
                    if (x>f->max) f->max=x;
                    const float was=f->hist[back];
                    /* an upshift: down 15..60% within 0.2 s while the car keeps going under throttle */
                    if (scan.samples>=SCAN_HISTORY && going && was>50.0f && x<was*0.85f && x>was*0.4f &&
                        (f->n_events==0 || scan.t-f->event_t[(f->n_events-1)%SCAN_EVENTS]>0.6f)) {
                        f->event_t[f->n_events%SCAN_EVENTS]=scan.t; ++f->n_events; ++f->drops;
                    }
                    f->hist[scan.hist_pos]=x;
                    if (speed>4.0f && x>0.0f) scan_ratio(f,x/speed);
                }
            }
            ScanInt *g=&o->i[k];
            if (g->ok) {
                const int32_t v=iv[k];
                if (v<-1 || v>9) g->ok=false;
                else {
                    if (scan.samples>0 && v!=g->last) {
                        ++g->changes;
                        if (v==g->last+1) { if (g->n_up<SCAN_EVENTS) g->up_t[g->n_up++]=scan.t; ++g->ups; }
                        if (g->changes>400) g->ok=false;   /* a counter or a flag, not a gear */
                    }
                    g->last=v; if (v<g->lo) g->lo=(int16_t)v; if (v>g->hi) g->hi=(int16_t)v;
                }
            }
        }
    }
    scan.speed_hist[scan.hist_pos]=speed;
    scan.hist_pos=(scan.hist_pos+1)%SCAN_HISTORY;
    ++scan.samples;
    if (scan.t>=scan.next_report && scan.reports<10) {
        ++scan.reports; scan.next_report=scan.t+30.0f;
        scan_report(scan.reports==10);
    }
}

/* The car stopped (or the race ended): the candidates so far, once, if there was enough driving. */
static void scan_stopped(void) {
    static float reported_at=-1.0f;
    if (scan.obj && scan.t>=30.0f && scan.t-reported_at>=30.0f && scan.reports<6) {
        reported_at=scan.t; ++scan.reports;
        scan_report(false);
    }
}

/* The gear from the engine's speed against the car's: each gear keeps its own ratio between them, so the ratios
 * seen while driving gather into a few groups, one a gear - the highest ratio first gear. A group counts once seen
 * for a while (clutch slip and wheelspin pass); the gear changes once the new one holds for a few samples. */
#define RIG_GEARS 10
static struct {
    float centre[RIG_GEARS]; unsigned hits[RIG_GEARS]; float last_s[RIG_GEARS]; int n;
    int current, pending, pending_n; float t;
} rig_gear;
static void rig_gear_reset(void) { memset(&rig_gear,0,sizeof rig_gear); rig_gear.current=-1; rig_gear.pending=-1; }
static float rig_gear_from_ratio(float engine, float speed, float dt) {
    rig_gear.t+=dt;
    if (!(engine>0.0f) || speed<4.0f) return rig_gear.current>=0 ? (float)(rig_gear.current+1) : -1.0f;
    const float ratio=engine/speed;
    int best=-1; float best_d=1e9f;
    for (int i=0;i<rig_gear.n;++i) { const float d=fabsf(ratio-rig_gear.centre[i])/rig_gear.centre[i]; if (d<best_d) { best_d=d; best=i; } }
    if (best>=0 && best_d<0.05f) {
        rig_gear.centre[best]+=(ratio-rig_gear.centre[best])*0.02f;
        ++rig_gear.hits[best]; rig_gear.last_s[best]=rig_gear.t;
    } else {
        /* a new group, or the oldest little-used one replaced */
        int at=rig_gear.n<RIG_GEARS ? rig_gear.n++ : -1;
        if (at<0) { unsigned least=~0u; for (int i=0;i<RIG_GEARS;++i) if (rig_gear.hits[i]<least) { least=rig_gear.hits[i]; at=i; } }
        rig_gear.centre[at]=ratio; rig_gear.hits[at]=1; rig_gear.last_s[at]=rig_gear.t;
        best=at;
    }
    /* groups that count: seen at least 20 times (a second) and a fair share of the drive - a clutch slipping or a
     * wheel spinning for a moment makes a group too, but a small one */
    unsigned total=0; for (int i=0;i<rig_gear.n;++i) total+=rig_gear.hits[i];
    const unsigned need=total/30>20 ? total/30 : 20;
    if (rig_gear.hits[best]<need) return rig_gear.current>=0 ? (float)(rig_gear.current+1) : -1.0f;
    int rank=0;
    for (int i=0;i<rig_gear.n;++i) if (i!=best && rig_gear.hits[i]>=need && rig_gear.centre[i]>rig_gear.centre[best]) ++rank;
    if (rank==rig_gear.current) { rig_gear.pending_n=0; return (float)(rank+1); }
    if (rank==rig_gear.pending) { if (++rig_gear.pending_n>=8) { rig_gear.current=rank; rig_gear.pending_n=0; } }
    else { rig_gear.pending=rank; rig_gear.pending_n=1; }
    return rig_gear.current>=0 ? (float)(rig_gear.current+1) : -1.0f;
}

/* the gears learnt so far, to the log with the status line */
static void rig_gear_report(void) {
    if (!scan.rpm_at || !rig_gear.n) return;
    unsigned total=0; for (int i=0;i<rig_gear.n;++i) total+=rig_gear.hits[i];
    const unsigned need=total/30>20 ? total/30 : 20;
    char line[256]; int n=snprintf(line,sizeof line,"Rig: gears learnt (rpm per km/h):");
    for (int g=0;g<rig_gear.n;++g) {
        int idx=-1; float best=-1.0f;   /* the g-th highest counted ratio */
        for (int i=0;i<rig_gear.n;++i) {
            if (rig_gear.hits[i]<need) continue;
            int rank=0; for (int j=0;j<rig_gear.n;++j) if (j!=i && rig_gear.hits[j]>=need && rig_gear.centre[j]>rig_gear.centre[i]) ++rank;
            if (rank==g) { idx=i; best=rig_gear.centre[i]; }
        }
        if (idx<0) break;
        n+=snprintf(line+n,sizeof line-n," %d: %.0f (%u samples)",g+1,best*scan.rpm_scale/3.6f,rig_gear.hits[idx]);
        if (n>=(int)sizeof line-24) break;
    }
    printf("%s\n",line);
}

/* what was sent last, for the trace */
static volatile float scan_sent_rpm=-1.0f, scan_sent_gear=-1.0f;
static void scan_peek(float *rpm, float *gear) { *rpm=scan_sent_rpm; *gear=scan_sent_gear; }

/* Until the engine's own is found: RPM and gear worked out from the speed and the throttle, as a six-speed gearbox
 * that shifts up near the red line under power (earlier on a light throttle) and down as the revs fall. The top
 * gear reaches the red line at the highest speed seen (at least 200 km/h). Not the game's own numbers: for shaker
 * effects (revs rising, a kick at each shift) and a rev counter that moves sensibly. */
static void rig_estimate(float *rpm, float *gear, float speed, float dt, float throttle, float brake) {
    static const float ratio[6]={3.6f,2.2f,1.55f,1.18f,0.95f,0.8f};   /* a typical six-speed, top gear last */
    if (!est.told) { est.told=true; printf("Rig: RPM and gear estimated from the speed and the throttle (the engine's own not found yet)\n"); }
    if (speed>est.top) est.top=speed;
    const float top=fmaxf(est.top*1.05f,200.0f/3.6f);
    if (!(dt>0.0f) || dt>0.5f) dt=0.0f;
    est.hold-=dt;
    /* the speed at the red line in gear g (1..6) */
    #define EST_V(g) (top*ratio[5]/ratio[(g)-1])
    float target;
    /* the shift buttons: pressed while driving, the gearbox is manual and the gear follows them from then on */
    unsigned up_n, down_n; runtime_pad_shifts(&up_n,&down_n);
    if (est.reset) { est.reset=false; est.manual_gear=1; est.up_seen=up_n; est.down_seen=down_n; }
    if (up_n!=est.up_seen || down_n!=est.down_seen) {
        if (speed>2.0f && up_n!=est.up_seen && !est.manual) {
            est.manual=true;
            if (!est.told_manual) { est.told_manual=true; printf("Rig: manual gearbox: the gear follows the shift buttons (Triangle up and Square down on a pad, the paddles on a wheel)\n"); }
        }
        /* counted from first gear at the start of the race, whether or not the gearbox was known to be manual yet */
        est.manual_gear+=(int)(up_n-est.up_seen)-(int)(down_n-est.down_seen);
        if (est.manual_gear<1) est.manual_gear=1;
        if (est.manual_gear>7) est.manual_gear=7;
        est.up_seen=up_n; est.down_seen=down_n;
    }
    /* an H-pattern shifter: its gear as it is (reverse -1, neutral 0) */
    int hgear;
    const bool shifter=runtime_pad_shifter(&hgear)>0;
    if (shifter && !est.told_manual) { est.told_manual=true; printf("Rig: the gear follows the H-pattern shifter\n"); }
    if (shifter && hgear<=0) {
        /* neutral: the revs follow the throttle; reverse: as first gear, backwards */
        const float r=hgear==0 || speed<0.5f ? EST_IDLE+throttle*(0.75f*EST_RED-EST_IDLE) : fmaxf(speed/EST_V(1)*EST_RED,EST_IDLE+throttle*0.35f*EST_RED);
        target=fminf(r,EST_RED);
        est.rpm+=(target-est.rpm)*fminf(1.0f,dt/0.08f);
        *rpm=est.rpm*rig_rpm_scale_cfg; *gear=(float)hgear;
        return;
    }
    if (est.manual || shifter) {
        const int g=shifter ? hgear : est.manual_gear;
        float v=g<=6 ? EST_V(g) : EST_V(6)*0.85f;
        if (est.red_v[g]>v) v=est.red_v[g];
        if (speed>v) { est.red_v[g]=speed; v=speed; }   /* past the model's red line: this car's gear is longer */
        float r=speed<0.5f ? EST_IDLE+throttle*(0.75f*EST_RED-EST_IDLE) : speed/v*EST_RED;
        if (g==1 && speed>=0.5f) r=fmaxf(r,EST_IDLE+throttle*0.35f*EST_RED);
        /* below idle in a high gear the clutch would be in: idle, rising with the throttle */
        if (r<EST_IDLE) r=EST_IDLE+throttle*0.1f*EST_RED;
        target=fminf(r,EST_RED);
        est.gear=g;
        est.rpm+=(target-est.rpm)*fminf(1.0f,dt/0.08f);
        *rpm=est.rpm*rig_rpm_scale_cfg; *gear=(float)g;
        return;
    }
    if (speed<0.5f) { est.gear=1; target=EST_IDLE+throttle*(0.75f*EST_RED-EST_IDLE); }
    else {
        float r=speed/EST_V(est.gear)*EST_RED;
        const float up_at=throttle>0.6f ? 0.93f*EST_RED : 0.6f*EST_RED;
        if (est.hold<=0.0f && est.gear<6 && r>up_at) { ++est.gear; est.hold=0.6f; }
        else if (est.hold<=0.0f && est.gear>1 && r<(brake>0.2f ? 0.45f : 0.3f)*EST_RED) { --est.gear; est.hold=0.6f; }
        r=speed/EST_V(est.gear)*EST_RED;
        /* pulling away in first: the clutch slips, the revs follow the throttle */
        if (est.gear==1) r=fmaxf(r,EST_IDLE+throttle*0.35f*EST_RED);
        target=fminf(fmaxf(r,EST_IDLE),EST_RED);
    }
    #undef EST_V
    /* the revs move fast but not in one step (a shift settles in about a tenth of a second) */
    est.rpm+=(target-est.rpm)*fminf(1.0f,dt/0.08f);
    *rpm=est.rpm*rig_rpm_scale_cfg; *gear=(float)est.gear;
}

/* The live RPM and gear, when known; estimated otherwise. speed (m/s) and dt for the gear from the ratio. */
static void scan_live(float *rpm, float *gear, float speed, float dt, float throttle, float brake) {
    *rpm=-1.0f; *gear=-1.0f;
    if (!scan.rpm_at) { rig_estimate(rpm,gear,speed,dt,throttle,brake); scan_sent_rpm=*rpm; scan_sent_gear=*gear; return; }
    float r;
    if (scan.rpm_at && rig_read(scan.rpm_at,&r,4) && isfinite(r) && r>=0.0f) {
        *rpm=r*scan.rpm_scale*rig_rpm_scale_cfg;
        if (*rpm>scan.rpm_max_seen) scan.rpm_max_seen=*rpm;
        if (!scan.gear_at) *gear=rig_gear_from_ratio(r,speed,dt);
    }
    int32_t g;
    if (scan.gear_at && rig_read(scan.gear_at,&g,4) && g>=-1 && g<=12) {
        if (scan.gear_type==2) g-=1;   /* reverse -1, neutral 0, first 1 */
        *gear=(float)g;
    }
    scan_sent_rpm=*rpm; scan_sent_gear=*gear;
}
