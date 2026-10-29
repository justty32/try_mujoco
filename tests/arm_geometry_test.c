#include <math.h>
#include <stdio.h>
#include <mujoco/mujoco.h>
#define CHECK(c,msg) do { if (!(c)) { fprintf(stderr,"FAIL: %s\n",msg); ok=0; } } while(0)
int main(int argc,char **argv) {
    if(argc!=2) return 2;
    char error[1024]; mjModel *m=mj_loadXML(argv[1],NULL,error,sizeof(error));
    if(!m) { fprintf(stderr,"%s\n",error); return 1; }
    mjData *d=mj_makeData(m); int ok=1;
    CHECK(m->nexclude==0 && !(m->opt.disableflags & mjDSBL_CONTACT),"self/ground collisions enabled");
    for(int g=0;g<m->ngeom;++g) CHECK(m->geom_contype[g]!=0 && m->geom_conaffinity[g]!=0,"all solids collide");
    for(int b=1;b<m->nbody;++b) {
        CHECK(isfinite(m->body_mass[b]) && m->body_mass[b]>0,"positive finite mass");
        for(int k=0;k<3;++k) CHECK(isfinite(m->body_inertia[3*b+k]) && m->body_inertia[3*b+k]>0,"positive finite inertia");
    }
    const double poses[][6]={{0,65,-100,0,35,0},{20,50,-85,25,25,-40},{-25,80,-110,-30,20,60},{0,65,-100,0,45,0},{0,0,0,0,0,0}};
    double minimum=1;
    for(int p=0;p<5;++p) {
        for(int j=0;j<6;++j) d->qpos[j]=poses[p][j]*mjPI/180;
        mj_forward(m,d); CHECK(d->ncon==0,"representative reachable pose has no engine contacts");
        for(int a=0;a<m->ngeom;++a) for(int b=a+1;b<m->ngeom;++b) {
            int ba=m->geom_bodyid[a],bb=m->geom_bodyid[b];
            /* Same/adjacent bodies intentionally overlap at simplified joints.
             * Fixed base and floor share a welded body. All other pairs checked. */
            if(m->body_weldid[ba]==m->body_weldid[bb] || m->body_parentid[ba]==bb || m->body_parentid[bb]==ba) continue;
            double dist=mj_geomDistance(m,d,a,b,1,NULL);
            if(dist<minimum) minimum=dist;
            if(dist < -1e-6) fprintf(stderr,"pose %d pair %s/%s dist=%g\n",p,mj_id2name(m,mjOBJ_GEOM,a),mj_id2name(m,mjOBJ_GEOM,b),dist);
            CHECK(dist>=-1e-6,"nonadjacent geometry clear in sampled pose");
        }
    }
    const double blocked[]={0,-60,0,0,0,0};
    for(int j=0;j<6;++j) d->qpos[j]=blocked[j]*mjPI/180;
    mj_forward(m,d); CHECK(d->ncon>0,"downward pose genuinely obstructed by base/floor");
    printf("arm_geometry: %s; 5 clear samples, minimum distance %.9f m; blocked contacts %d\n",ok?"ok":"FAILED",minimum,d->ncon);
    mj_deleteData(d);mj_deleteModel(m);return ok?0:1;
}
