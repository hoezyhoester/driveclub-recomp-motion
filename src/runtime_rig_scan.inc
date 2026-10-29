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
    int best=-1; double best_r=0.5;
    for (int i=0;i<rig_test.n;++i) {
        const double mx=rig_test.sx[i]/n, vx=rig_test.sxx[i]/n-mx*mx, cov=rig_test.sxy[i]/n-mx*my;
        const double r=vx>1e-6 && vy>1e-6 ? cov/sqrt(vx*vy) : 0.0;
        const bool moving=rig_test.load_max[i]-rig_test.load_min[i]>100.0f;
        printf("Rig: tyres at %s: loads %.0f..%.0f N, lateral force against the car's turning r=%.2f%s\n",rig_test.c[i].how,
               rig_test.load_min[i],rig_test.load_max[i],r,moving && r>0.5 ? "" : " - not live");
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

#define SCAN_OBJECTS 64
#define SCAN_BYTES 0x800
#define SCAN_FIELDS (SCAN_BYTES/4)
#define SCAN_HISTORY 4        /* samples back the drop is measured over (0.2 s at 20 Hz) */
#define SCAN_EVENTS 24

typedef struct {
    float min, max, hist[SCAN_HISTORY];
    uint16_t drops, n_events;
    float event_t[SCAN_EVENTS];   /* drop times (s) */
    bool in_range;
} ScanFloat;
typedef struct {
    int32_t last; int16_t lo, hi; uint16_t ups, changes; bool ok;
    float up_t[SCAN_EVENTS]; uint16_t n_up;
    uint32_t seen;               /* the values seen, as bits -1..30 */
} ScanInt;
typedef struct {
    uintptr_t address; char label[48];
    ScanFloat f[SCAN_FIELDS];
    ScanInt i[SCAN_FIELDS];
} ScanObject;

static struct {
    uintptr_t vehicle, state;    /* what was scanned: a new car starts again */
    ScanObject *obj; int n_obj;
    unsigned samples, hist_pos; float drive_s;
    float speed_hist[SCAN_HISTORY];
    float t;                     /* driving time (s) */
    int reports; float next_report;
    /* what is used */
    uintptr_t rpm_at; float rpm_scale; uintptr_t gear_at; int gear_type;   /* 0 int32, 1 float */
    char rpm_label[64], gear_label[64];
    float rpm_max_seen;
} scan;

static void scan_reset(uintptr_t vehicle, uintptr_t state) {
    if (!scan.obj) scan.obj=calloc(SCAN_OBJECTS,sizeof(ScanObject));
    if (!scan.obj) return;
    memset(scan.obj,0,SCAN_OBJECTS*sizeof(ScanObject));
    scan.vehicle=vehicle; scan.state=state; scan.n_obj=0; scan.samples=0; scan.hist_pos=0; scan.t=0;
    scan.reports=0; scan.next_report=30.0f;
    { const char *e=getenv("BB_MOTION_SCAN_FIRST"); if (e && atof(e)>0) scan.next_report=(float)atof(e); }   /* tests */
    scan.rpm_at=0; scan.gear_at=0; scan.rpm_max_seen=0;
    /* the objects: the car, its state, and what the car (its first 0x400 bytes) and the state (0x100) point to */
    const uintptr_t roots[2]={vehicle,state}; const char *names[2]={"vehicle","state"};
    for (int r=0;r<2 && scan.n_obj<SCAN_OBJECTS;++r) {
        if (!roots[r]) continue;
        ScanObject *o=&scan.obj[scan.n_obj++]; o->address=roots[r]; snprintf(o->label,sizeof o->label,"%s",names[r]);
    }
    for (int r=0;r<2;++r) {
        if (!roots[r]) continue;
        uint64_t ptrs[0x1000/8]; const int n=r==0 ? 0x1000/8 : 0x100/8;
        if (!rig_read(roots[r],ptrs,n*8)) continue;
        for (int k=0;k<n && scan.n_obj<SCAN_OBJECTS;++k) {
            if (!rig_pointer(ptrs[k]) || ptrs[k]==vehicle || ptrs[k]==state) continue;
            bool dup=false; for (int m=0;m<scan.n_obj;++m) if (scan.obj[m].address==ptrs[k]) dup=true;
            uint8_t probe[SCAN_BYTES];
            if (dup || !rig_read(ptrs[k],probe,sizeof probe)) continue;
            ScanObject *o=&scan.obj[scan.n_obj++]; o->address=(uintptr_t)ptrs[k];
            snprintf(o->label,sizeof o->label,"%s+0x%x ->",names[r],k*8);
        }
    }
    for (int m=0;m<scan.n_obj;++m) for (int k=0;k<SCAN_FIELDS;++k) {
        scan.obj[m].f[k].in_range=true; scan.obj[m].f[k].min=1e30f; scan.obj[m].f[k].max=-1e30f;
        scan.obj[m].i[k].ok=true; scan.obj[m].i[k].lo=32767; scan.obj[m].i[k].hi=-32768;
    }
    printf("Rig: looking for the engine's RPM and gear in %d objects of the car; drive a few minutes with gear changes\n",scan.n_obj);
}

/* a field's value as RPM: rpm itself (300..16000) or rad/s (30..1700) */
static bool scan_rpm_range(float lo, float hi, float *scale) {
    if (lo>=300.0f && hi<=16000.0f && hi-lo>=1500.0f) { *scale=1.0f; return true; }
    if (lo>=30.0f && hi<=1700.0f && hi-lo>=150.0f) { *scale=60.0f/(2.0f*3.14159265f); return true; }
    return false;
}

/* drops of field f matched by steps up of gear g, within 0.4 s */
static int scan_match(const ScanFloat *f, const ScanInt *g) {
    int m=0;
    for (int a=0;a<g->n_up && a<SCAN_EVENTS;++a)
        for (int b=0;b<f->n_events && b<SCAN_EVENTS;++b)
            if (fabsf(g->up_t[a]-f->event_t[b])<0.4f) { ++m; break; }
    return m;
}

static void scan_report(bool final) {
    typedef struct { int o, k; int score; float scale; int matched; } Cand;
    Cand rpm[6]={{0}}; int n_rpm=0;
    for (int m=0;m<scan.n_obj;++m) for (int k=0;k<SCAN_FIELDS;++k) {
        const ScanFloat *f=&scan.obj[m].f[k]; float scale;
        if (!f->in_range || f->drops<3 || !scan_rpm_range(f->min,f->max,&scale)) continue;
        Cand c={m,k,f->drops,scale,0};
        int at=n_rpm<6 ? n_rpm++ : 5;
        if (at==5 && n_rpm==6 && rpm[5].score>=c.score) continue;
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
        int at=n_gear<6 ? n_gear++ : 5;
        if (at==5 && n_gear==6 && gear[5].score>=c.score) continue;
        gear[at]=c;
        for (int x=at;x>0 && gear[x].score>gear[x-1].score;--x) { Cand t=gear[x]; gear[x]=gear[x-1]; gear[x-1]=t; }
    }
    printf("Rig: after %.0f s of driving, %d RPM and %d gear candidates%s\n",scan.t,n_rpm,n_gear,final ? " (final)" : "");
    for (int r=0;r<n_rpm;++r) {
        const ScanObject *o=&scan.obj[rpm[r].o]; const ScanFloat *f=&o->f[rpm[r].k];
        printf("Rig: RPM candidate %d: %s +0x%x, %.0f..%.0f%s, %u drops\n",r+1,o->label,rpm[r].k*4,f->min,f->max,
               rpm[r].scale==1.0f ? " rpm" : " rad/s",(unsigned)f->drops);
    }
    for (int r=0;r<n_gear;++r) {
        const ScanObject *o=&scan.obj[gear[r].o]; const ScanInt *g=&o->i[gear[r].k];
        printf("Rig: gear candidate %d: %s +0x%x, %d..%d, %u steps up, %d at RPM drops\n",r+1,o->label,gear[r].k*4,
               (int)g->lo,(int)g->hi,(unsigned)g->ups,gear[r].matched);
    }
    /* one that stands out is used now: twice the drops of the next, or the only one; a gear only at those drops */
    if (n_rpm>0 && rpm[0].score>=5 && (n_rpm==1 || rpm[0].score>=2*rpm[1].score)) {
        const ScanObject *o=&scan.obj[rpm[0].o];
        scan.rpm_at=o->address+rpm[0].k*4; scan.rpm_scale=rpm[0].scale;
        snprintf(scan.rpm_label,sizeof scan.rpm_label,"%s +0x%x",o->label,rpm[0].k*4);
        printf("Rig: using RPM from %s\n",scan.rpm_label);
    }
    if (scan.rpm_at && n_gear>0 && gear[0].matched>=3) {
        const ScanObject *o=&scan.obj[gear[0].o];
        scan.gear_at=o->address+gear[0].k*4; scan.gear_type=0;
        snprintf(scan.gear_label,sizeof scan.gear_label,"%s +0x%x",o->label,gear[0].k*4);
        printf("Rig: using the gear from %s\n",scan.gear_label);
    }
}

/* One sample (the idle thread, 20 times a second) while the car is driven. */
static void scan_sample(uintptr_t vehicle, uintptr_t state, float speed, float throttle, float dt) {
    if (vehicle!=scan.vehicle || state!=scan.state) scan_reset(vehicle,state);
    if (!scan.obj || !scan.n_obj) return;
    const bool driving=speed>3.0f;
    if (!driving) return;
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

/* The live RPM and gear, when known: -1 otherwise. */
static void scan_live(float *rpm, float *gear) {
    *rpm=-1.0f; *gear=-1.0f;
    float r;
    if (scan.rpm_at && rig_read(scan.rpm_at,&r,4) && isfinite(r) && r>=0.0f) {
        *rpm=r*scan.rpm_scale;
        if (*rpm>scan.rpm_max_seen) scan.rpm_max_seen=*rpm;
    }
    int32_t g;
    if (scan.gear_at && rig_read(scan.gear_at,&g,4) && g>=-1 && g<=9) *gear=(float)g;
}
