#include <math.h>
#include <stdio.h>
#include <mujoco/mujoco.h>
#define CHECK(c,msg) do { if (!(c)) { fprintf(stderr,"FAIL: %s\n",msg); ok=0; } } while(0)
int main(int argc,char **argv) {
    if(argc!=3) return 2;
    char error[1024];
    mjModel *m=mj_loadXML(argv[1],NULL,error,sizeof(error));
    if(!m) { fprintf(stderr,"%s\n",error); return 1; }
    mjModel *p=mj_loadXML(argv[2],NULL,error,sizeof(error));
    if(!p) { fprintf(stderr,"%s\n",error); mj_deleteModel(m); return 1; }
    int ok=1;
    CHECK(m->njnt==6 && m->nu==6 && m->nq==6 && m->nv==6 && m->nbody==8,"6R dimensions");
    CHECK(m->ngeom==10 && p->ngeom==m->ngeom,"nine cylinders and floor");
    const char *names[]={"J1_base_yaw","J2_shoulder_pitch","J3_elbow_pitch","J4_wrist_roll","J5_wrist_pitch","J6_tool_roll"};
    const double axes[6][3]={{0,0,1},{0,-1,0},{0,-1,0},{1,0,0},{0,-1,0},{1,0,0}};
    const double gears[]={12,20,12,3,3,2}, home[]={0,65,-100,0,35,0};
    for(int j=0;j<6 && j<m->njnt;++j) {
        CHECK(mj_name2id(m,mjOBJ_JOINT,names[j])==j,"joint names/order");
        CHECK(m->jnt_type[j]==mjJNT_HINGE && m->actuator_trnid[2*j]==j,"hinge motor mapping");
        CHECK(m->actuator_ctrllimited[j] && m->actuator_gear[6*j]==gears[j] && m->actuator_ctrlrange[2*j]==-1 && m->actuator_ctrlrange[2*j+1]==1,"explicit torque rating");
        CHECK(m->jnt_limited[j],"joint limits enabled");
        if(j!=4) for(int k=0;k<2;++k) CHECK(m->jnt_range[2*j+k]==p->jnt_range[2*j+k],"other proposal limits preserved");
        for(int k=0;k<3;++k) CHECK(fabs(m->jnt_axis[3*j+k]-axes[j][k])<1e-12,"6R local axes");
    }
    CHECK(m->nbody==p->nbody,"approved body count");
    for(int b=0;b<m->nbody && b<p->nbody;++b) {
        CHECK(m->body_parentid[b]==p->body_parentid[b],"approved parent chain");
        for(int k=0;k<3;++k) CHECK(fabs(m->body_pos[3*b+k]-p->body_pos[3*b+k])<1e-12,"approved body position");
        for(int k=0;k<4;++k) CHECK(fabs(m->body_quat[4*b+k]-p->body_quat[4*b+k])<1e-12,"approved body orientation");
    }
    for(int g=0;g<m->ngeom && g<p->ngeom;++g) {
        CHECK(m->geom_type[g]==p->geom_type[g] && m->geom_bodyid[g]==p->geom_bodyid[g],"approved primitive/body");
        for(int k=0;k<3;++k) {
            CHECK(fabs(m->geom_size[3*g+k]-p->geom_size[3*g+k])<1e-12,"approved cylinder dimensions");
            CHECK(fabs(m->geom_pos[3*g+k]-p->geom_pos[3*g+k])<1e-12,"approved cylinder position");
        }
        for(int k=0;k<4;++k) {
            CHECK(fabs(m->geom_quat[4*g+k]-p->geom_quat[4*g+k])<1e-12,"approved cylinder orientation");
            CHECK(m->geom_rgba[4*g+k]==p->geom_rgba[4*g+k],"approved colors");
        }
    }
    CHECK(fabs(m->jnt_range[8]+mjPI/4)<1e-12 && fabs(m->jnt_range[9]-mjPI/4)<1e-12,"J5 conservative physical travel");
    mjData *d=mj_makeData(m);
    int key=mj_name2id(m,mjOBJ_KEY,"home"), site=mj_name2id(m,mjOBJ_SITE,"tool_center");
    CHECK(key>=0 && site>=0,"home and tool site exist");
    if(key>=0 && site>=0) {
        mj_resetDataKeyframe(m,d,key); mj_forward(m,d);
        for(int j=0;j<6;++j) CHECK(fabs(d->qpos[j]*180/mjPI-home[j])<1e-10,"home radians decode to approved degrees");
        CHECK(d->ncon==0,"home contact free");
        for(int j=0;j<6;++j) CHECK(fabs(d->qfrc_bias[j])<gears[j],"home gravity inside motor capacity");
        double pos[3]; for(int k=0;k<3;++k) pos[k]=d->site_xpos[3*site+k];
        CHECK(fabs(d->site_xmat[9*site]-1)<1e-12,"home tool axis points world +X");
        d->qpos[5]+=mjPI/4; mj_forward(m,d);
        for(int k=0;k<3;++k) CHECK(fabs(d->site_xpos[3*site+k]-pos[k])<1e-12,"J6 roll keeps tool center");
        CHECK(fabs(d->site_xmat[9*site+4]-sqrt(.5))<1e-12 && fabs(d->site_xmat[9*site+7]-sqrt(.5))<1e-12,"J6 roll changes tool orientation");
    }
    printf("six_axis_arm_model: %s\n",ok?"ok":"FAILED");
    mj_deleteData(d); mj_deleteModel(p); mj_deleteModel(m); return ok?0:1;
}
