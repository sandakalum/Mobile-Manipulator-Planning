// host define.h
#pragma once

#define MAX_CONNECT 6
#define PI 3.14159265358979323846
#define MAX_DATA_SIZE_LRF 1500//726//727//1081//726//1081//727//1081//700
#define MAX_TRACKING_OBJECT 10//99
#define MAX_PARTICLE_NUM 2000//2000//8000
#define MAX_PARTICLE_NUM_MCMC 1000//1000
#define TRACKING_HISTORY 30
#define STAGE_X 8000//11000//8000//12000//9000//12000//14000
#define STAGE_Y 8000//11000//8000//12000//11000//18000//20000
#define GRID_DISTANCE 200//500
#define NEAR_POSITION 0.5 // [m]
#define CNT_PER_TMSSEND 50
#define MSEC_PER_TMSSEND 100//200
#define CNT_PER_NODEIMAGESAVE 30
#define OCCLUDED_CNT 20
#define DISCOVER_CNT 5

#define sqr(x) ((x)*(x))
#define ssqrs(x,y) (sqr(x)+sqr(y))
#define norm(x,y,z) sqrt(sqr(x)+sqr(y)+sqr(z))
#define deg2rad(x) ((double)x*PI/180.0)

typedef struct _LRFParam
{
    double tx;
    double ty;
    double tz;
    double rx;
    double ry;
    double rz;
    int step;
    double viewangle;
    double divangle;
} LRFParam;

typedef struct
{
    double x;
    double y;
} pos;

typedef struct
{
    int id;
    int flag;
    int count;
    pos start_pos;
    pos tmp_pos;
    pos end_pos;
    pos vector;
} IDparam;

typedef struct
{
    int id;
    int flag;
    double x;
    double y;
} Psenparam;

#define ZeroMemory(arg1, arg2) memset(arg1, 0, arg2)
#define CopyMemory(arg1, arg2, arg3) memcpy(arg1, arg2, arg3)
