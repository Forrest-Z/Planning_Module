#pragma once
#include "ros/ros.h"
#include "Path/PathMaker.cpp"
#include "DataWarehouse.cpp"
#include "EventHandler.cpp"
#include "Control_pkg/msg2DVelodyne.h"
#include "Control_pkg/msgsick_vector.h"
#include "Control_pkg/msgVelodyne.h"
#include "Control_pkg/gps.h"
#include "Control_pkg/line_function.h"
#include "Control_pkg/path.h"
#include "Control_pkg/lane.h"
#include "Control_pkg/GPSMessage.h"
#include "Control_pkg/VehicleStatus.h"
#include <std_msgs/String.h>
#include "Machine/StateMachine.cpp"
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Point.h>
#include "tf2_msgs/TFMessage.h"
#include "cmath"
#include "nav_msgs/Odometry.h"
#include "UTM.cpp"
#include "fstream"
#include"visualization_msgs/MarkerArray.h"
#include"visualization_msgs/Marker.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2/LinearMath/Quaternion.h>


#include <stdio.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <float.h>
#include <stdlib.h>

// #include<windows.h>
ros::Publisher coef1;
ros::Publisher coef2;
ros::Publisher coef3;
ros::Publisher coef4;
#define MAX_SPEED 50.0 / 3.6   //maximum speed [m/s]
#define MAX_ACCEL 2.0   //maximum acceleration [m/ss]
#define MAX_CURVATURE 1.0   //maximum curvature [1/m]
#define MAX_ROAD_WIDTH 7.0   //maximum road width [m]
#define D_ROAD_W 1.0   //road width sampling length [m]
#define DT 0.2   //time tick [s]
#define MAX_T 5.0   //max prediction time [m]
#define MIN_T 4.0   //min prediction time [m]
#define TARGET_SPEED 30.0 / 3.6   //target speed [m/s]
#define D_T_S 5.0 / 3.6   //target speed sampling length [m/s]
#define N_S_SAMPLE 1   //sampling number of target speed
#define ROBOT_RADIUS 2.0   //robot radius [m]
#define SquaredSum(x,y) ((x)*(x)+(y)*(y))
#define min(a,b)  (((a) < (b)) ? (a) : (b))
#define K_J 0.1
#define K_T 0.1
#define K_D 1.0
#define K_LAT 1.0
#define K_LON 1.0
#define MAX_PATH 300
#define T_LEN 30 //((MAX_T-MIN_T)/DT)
#define MAX_D 10

void linsolve(float* _A, float* b, float* _x, int demesion)
{
	int i, j, k;
	float A[MAX_D][MAX_D], c, sum = 0.0;

	for (i = 0; i < demesion; i++)
	{
		for (j = 0; j < (demesion + 1); j++)
		{
			if (j == demesion)
				A[i][j] = b[i];
			else
				A[i][j] = _A[i * demesion + j];
		}
	}
	for (j = 0; j < demesion; j++) /* loop for the generation of upper triangular matrix*/
	{
		for (i = 0; i < demesion; i++)
		{
			if (i > j)
			{
				c = A[i][j] / A[j][j];
				for (k = 0; k < demesion + 1; k++)
				{
					A[i][k] = A[i][k] - c * A[j][k];
				}
			}
		}
	}
	_x[demesion - 1] = A[demesion - 1][demesion] / A[demesion - 1][demesion - 1];
	/* self loop is for backward substitution*/
	for (i = demesion - 2; i >= 0; i--)
	{
		sum = 0;
		for (j = i + 1; j < demesion; j++)
		{
			sum = sum + A[i][j] * _x[j];
		}
		_x[i] = (A[i][demesion] - sum) / A[i][i];
	}
	return;
}

typedef struct _Spline {
	float a[100];
	float b[100];
	float c[100];
	float d[100];
	float w[100];
	float x[100];
	float y[100];
	float Amat[20][20];
	float Bmat[20];
	int nx;
	int size;

	float (*calc_A)(struct _Spline* self, float*);
	float (*calc_B)(struct _Spline* self, float*);
	float (*calc)(struct _Spline* self, float);
	float (*calcd)(struct _Spline* self, float);
	float (*calcdd)(struct _Spline* self, float);
	int (*search_index)(struct _Spline* self, float);
	void (*GetCoef)(struct _Spline* self, float*);
}Spline;

typedef struct _Spline2D {
	float s[100];
	Spline* sx;
	Spline* sy;
	int size;
	float (*calc_position_x)(struct _Spline2D*, float);
	float (*calc_position_y)(struct _Spline2D*, float);
	float (*calc_curvature)(struct _Spline2D*, float);
	float (*calc_yaw)(struct _Spline2D*, float);

}Spline2D;

int binsearch(float data[], int n, float key) {
	int low, high;
	int mid;

	low = 0;
	high = n - 1;
	while (low < high) {
		mid = (low + high) / 2;
		if (data[mid] <= key)
			low = mid + 1;
		else
			high = mid;
	}
	return low;
}

int search_index(Spline* self, float x) {
	return binsearch(self->x, self->nx, x)-1;
}

float calc_A(Spline* self, float* h) {

	for (int i = 0; i < self->nx; i++)
		for (int j = 0; j < self->nx; j++)
			self->Amat[i][j] = 0;
	self->Amat[0][0] = 1;
	
	for (int i = 0; i < self->nx - 1; i++) {
		if (i != self->nx - 2)
			self->Amat[i + 1][i + 1] = 2 * (h[i] + h[i + 1]);

		self->Amat[i + 1][i] = h[i];
		self->Amat[i][i + 1] = h[i];
	}
	self->Amat[0][1] = 0;
	self->Amat[self->nx - 1][self->nx - 2] = 0;
	self->Amat[self->nx - 1][self->nx - 1] = 1;
	return 0;
}
float calc_B(Spline* self, float* h) {
	for (int i = 0; i < self->nx; i++)
		self->Bmat[i] = 0;
	for (int i = 0; i < self->nx - 2; i++)
		self->Bmat[i + 1] = 3 * (self->a[i + 2] - self->a[i + 1]) / h[i + 1] - 3 * (self->a[i + 1] - self->a[i]) / h[i];

	return 0;
}
float calc(Spline* self, float t) {
	if (t < self->x[0]) {
		return -1;
	}
	else if (t > self->x[self->nx - 1]) {
		return -1;
	}

	int i = self->search_index(self, t);
	float dx = t - self->x[i];
	float result = self->a[i] + self->b[i] * dx + self->c[i] * dx * dx + self->d[i] * dx * dx * dx;
	return result;
}
float calcd(Spline* self, float t) {
	if (t < self->x[0]) {
		return -1;
	}
	else if (t > self->x[self->nx - 1]) {
		return -1;
	}
	int i = search_index(self, t); //
	float dx = t - self->x[i];
	float result = self->b[i] + 2.0 * self->c[i] * dx + 3.0 * self->d[i] * dx * dx;
	return result;
}
float calcdd(Spline* self, float t) {
	if (t < self->x[0]) {
		return -1;
	}
	else if (t > self->x[self->nx - 1]) {
		return -1;
	}
	int i = search_index(self, t); //
	float dx = t - self->x[i];
	float result = 2.0 * self->c[i] + 6.0 * self->d[i] * dx;
	return result;
}

void _GetCoef(Spline* self, float* co)
{
	co[0] = self->d[0];
	co[1] = -3 * self->d[0] * self->x[0] + self->c[0];
	co[2] = 3 * self->d[0] * self->x[0] * self->x[0] - 2 * self->x[0] * self->c[0] + self->b[0];
	co[3] = -self->d[0] * self->x[0] * self->x[0] * self->x[0] + self->c[0] * self->x[0] * self->x[0] - self->b[0] * self->x[0] + self->a[0];
}

Spline* init_spline(float* x, float* y, int size) {

	Spline* self =(Spline*)malloc(sizeof(Spline));
	self->size = size;
	self->calc_A = calc_A;
	self->calc_B = calc_B;
	self->calc = calc;
	self->calcd = calcd;
	self->calcdd = calcdd;
	self->search_index = search_index;
	self->GetCoef = _GetCoef;

	for (int i = 0; i < size; i++) {
		self->x[i] = x[i];
		self->y[i] = y[i];
		self->a[i] = self->y[i];
	}
	self->nx = size;

	float h[100];

	for (int i = 0; i < self->nx-1; i++)
		h[i] = x[i+1] - x[i];
	
	calc_A(self, h);
	calc_B(self, h);

	float* A = (float*)malloc(sizeof(float)* self->nx * self->nx);
	
	for (int i = 0; i < self->nx; i++)
		for(int j=0; j < self->nx; j++)
			A[i * self->nx + j] = self->Amat[i][j];

	linsolve(A, self->Bmat, self->c, self->nx);
	free(A);

	for (int i = 0; i < self->nx - 1; i++) {
		self->d[i] = ((self->c[i + 1] - self->c[i]) / (3 * h[i]));
		self->b[i] = (self->a[i + 1] - self->a[i]) / h[i] - h[i] * (self->c[i + 1] + 2.0 * self->c[i]) / 3.0;
	}
	return self;
}
float _cal_position_x(Spline2D* self, float s) {
	return calc(self->sx, s);
}
float _cal_position_y(Spline2D* self, float s) {
	return calc(self->sy, s);
}
float _calc_curvature(Spline2D* self, float s) {
	float dx = calcd(self->sx, s);
	float ddx = calcdd(self->sx, s);
	float dy = calcd(self->sy, s);
	float ddy = calcdd(self->sy, s);
	double temp = pow((dx * dx + dy * dy), (3.0 / 2.0));
	float k = (ddy * dx - ddx * dy) / temp;

	return k;
}
float _calc_yaw(Spline2D* self, float s) {
	float dx = calcd(self->sx, s);
	float dy = calcd(self->sy, s);
	float yaw = atan2(dy, dx);
	return yaw;
}
Spline2D* init(float* x, float* y, int size) {
	Spline2D* self = (Spline2D*)malloc(sizeof(Spline2D));

	self->calc_position_x = _cal_position_x;
	self->calc_position_y = _cal_position_y;
	self->calc_curvature = _calc_curvature;
	self->calc_yaw = _calc_yaw;

	//---------------------------INIT--------------------
	float dx[100];
	float dy[100];
	float ds[100];

	self->size = size;
	
	for (int i = 0; i < size - 1; i++) {
		dx[i] = x[i + 1] - x[i];
		dy[i] = y[i + 1] - y[i];
		ds[i] = sqrt((dx[i] * dx[i]) + (dy[i] * dy[i]));
	}
	self->s[0] = 0;
	
	for (int i = 1; i < size; i++)
		self->s[i] = self->s[i-1] + ds[i-1];

	self->sx = init_spline(self->s, x, size);
	self->sy = init_spline(self->s, y, size);
	return self;
}


typedef struct _QuinticPolynomial
{
	float a0, a1, a2, a3, a4, a5;
	float A[3][3];
	float b[3];
	float x[3];
	float (*calc_point)(struct _QuinticPolynomial*, float);
	float (*calc_first_derivative)(struct _QuinticPolynomial*, float);
	float (*calc_second_derivative)(struct _QuinticPolynomial*, float);
	float (*calc_third_derivative)(struct _QuinticPolynomial*, float);
}QuinticPolynomial;

float Quintic_calc_point(QuinticPolynomial* self, float t)
{
	return self->a0 + self->a1 * t + self->a2 * pow(t, 2) + self->a3 * pow(t, 3) + self->a4 * pow(t, 4) + self->a5 * pow(t, 5);
}


float Quintic_calc_first_derivative(QuinticPolynomial* self, float t)
{
	return self->a1 + 2 * self->a2 * t + 3 * self->a3 * pow(t, 2) + 4 * self->a4 * pow(t, 3) + 5 * self->a5 * pow(t, 4);
}

float Quintic_calc_second_derivative(QuinticPolynomial* self, float t)
{
	return  2 * self->a2 + 6 * self->a3 * t + 12 * self->a4 * pow(t, 2) + 20 * self->a5 * pow(t, 3);
}

float Quintic_calc_third_derivative(QuinticPolynomial* self, float t)
{
	return 6 * self->a3 + 24 * self->a4 * t + 60 * self->a5 * pow(t, 2);
}


QuinticPolynomial* new_QuinticPolynomial(float xs, float vxs, float axs, float xe, float vxe, float axe, float time) {
	QuinticPolynomial* self = (QuinticPolynomial*)malloc(sizeof(QuinticPolynomial));
	int i = 0, j = 0, aa;
	float x[3];
	self->a0 = xs;
	self->a1 = vxs;
	self->a2 = axs / 2.0;
	self->calc_point = Quintic_calc_point;
	self->calc_first_derivative = Quintic_calc_first_derivative;
	self->calc_second_derivative = Quintic_calc_second_derivative;
	self->calc_third_derivative = Quintic_calc_third_derivative;
	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++)
		{
			aa = i == 0 ? 1 : i == 1 ? (3 + j) : (3 + j) * (2 + j);
			self->A[i][j] = aa * pow(time, (3 + j) - i);
		}
	}
	self->b[0] = xe - self->a0 - self->a1 * time - self->a2 * pow(time, 2);
	self->b[1] = vxe - self->a1 - 2 * self->a2 * time;
	self->b[2] = axe - 2 * self->a2;
	linsolve((float*)self->A, self->b, x, 3);
	self->a3 = x[0];
	self->a4 = x[1];
	self->a5 = x[2];
	return self;
}

typedef struct _QuarticPolynomial
{
    float a0, a1, a2, a3, a4;
    float A[2][2];
    float b[2];
    float x[2];
    float (*calc_point)(struct _QuarticPolynomial*, float);
    float (*calc_first_derivative)(struct _QuarticPolynomial*, float);
    float (*calc_second_derivative)(struct _QuarticPolynomial*, float);
    float (*calc_third_derivative)(struct _QuarticPolynomial*, float);
}QuarticPolynomial;


float Quartic_calc_point(QuarticPolynomial* self, float t)
{
	return self->a0 + self->a1 * t + self->a2 * pow(t, 2) + self->a3 * pow(t, 3) + self->a4 * pow(t, 4);
}


float Quartic_calc_first_derivative(QuarticPolynomial* self, float t)
{
	return self->a1 + 2 * self->a2 * t + 3 * self->a3 * pow(t, 2) + 4 * self->a4 * pow(t, 3);
}

float Quartic_calc_second_derivative(QuarticPolynomial* self, float t)
{
	return  2 * self->a2 + 6 * self->a3 * t + 12 * self->a4 * pow(t, 2);
}

float Quartic_calc_third_derivative(QuarticPolynomial* self, float t)
{
	return 6 * self->a3 + 24 * self->a4 * t;
}

QuarticPolynomial* new_QuarticPolynomial(float xs, float vxs, float axs, float vxe, float axe, float time) {
    QuarticPolynomial* self = (QuarticPolynomial*)malloc(sizeof(QuarticPolynomial));
    float x[2];
    self->a0 = xs;
    self->a1 = vxs;
    self->a2 = axs / 2.0;

    self->calc_point = Quartic_calc_point;
    self->calc_first_derivative = Quartic_calc_first_derivative;
    self->calc_second_derivative = Quartic_calc_second_derivative;
    self->calc_third_derivative = Quartic_calc_third_derivative;
   
    self->A[0][0] = 3 * pow(time,2);
    self->A[0][1] = 4 * pow(time,3);
    self->A[1][0] = 6 * time;
    self->A[1][1] = 12 * pow(time,2);
    
    self->b[0] = vxe - self->a1 - 2 * self->a2 * time;
    self->b[1] = axe - 2*self->a2;
    linsolve((float*)self->A,self->b,x,2);
    self->a3 = x[0];
    self->a4 = x[1];
    return self;
}

typedef struct _FrenetPath
{
    int limit;
    float t[T_LEN];
    float d[T_LEN];
    float d_d[T_LEN];
    float d_dd[T_LEN];
    float d_ddd[T_LEN];
    float s[T_LEN];
    float s_d[T_LEN];
    float s_dd[T_LEN];
    float s_ddd[T_LEN];
    float cd;
    float cv;
    float cf;
    float x[T_LEN];
    float y[T_LEN];
    float yaw[T_LEN];
    float ds[T_LEN];
    float c[T_LEN];
}FrenetPath;

void Set_FrenetPath(FrenetPath* self, QuinticPolynomial *lat_qp, QuarticPolynomial *lon_qp)
{
    int i;
    for(i=0; i< self->limit;i++)
    {
        self->t[i] = DT*i;
        self->d[i] = lat_qp->calc_point(lat_qp, DT*i);
        self->d_d[i] = lat_qp->calc_first_derivative(lat_qp, DT*i);
        self->d_dd[i] = lat_qp->calc_second_derivative(lat_qp, DT*i);
        self->d_ddd[i] = lat_qp->calc_third_derivative(lat_qp, DT*i);
        self->s[i] = lon_qp->calc_point(lon_qp, DT*i);
        self->s_d[i] = lon_qp->calc_first_derivative(lon_qp, DT*i);
        self->s_dd[i] = lon_qp->calc_second_derivative(lon_qp, DT*i);
        self->s_ddd[i] = lon_qp->calc_third_derivative(lon_qp, DT*i);
    }
}

int calc_frenet_paths(FrenetPath* paths, float c_speed, float c_d, float c_d_d, float c_d_dd, float s0)
{
    int i=0,j=0;
    float Ti,di,tv;
    int left_road_width, right_road_width;
    float target_speed;
    float Jp, Js, ds;
    //left_road_width = Getleftroadwidth();
    //right_road_width = Getrightroadwidth();
    // target_speed = Gettarget_speed();
	left_road_width = 7;
	right_road_width = 7;
	target_speed = 30.0 / 3.6;
    QuinticPolynomial *lat_qp;
    for(di = -left_road_width; di < right_road_width; di+=D_ROAD_W)
    {
        for(Ti = MIN_T; Ti < MAX_T-0.0001; Ti += DT)
        {
            lat_qp = new_QuinticPolynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti);
            for(tv = target_speed - D_T_S * N_S_SAMPLE; tv<=(target_speed + D_T_S * N_S_SAMPLE)*1.0001;tv+=D_T_S)
            {
                paths[i].limit = Ti/DT;
                QuarticPolynomial* lon_qp = new_QuarticPolynomial(s0,c_speed,0,tv,0,Ti);
                Set_FrenetPath(&(paths[i]), lat_qp, lon_qp);
                free(lon_qp);
                Jp = 0;
                Js = 0;
                for(j=0;j< paths[i].limit;j++)
                {
                    Jp+=pow(paths[i].d_ddd[j],2);
                    Js+=pow(paths[i].s_ddd[j],2);
                }
                ds = pow((target_speed - paths[i].s_d[paths[i].limit-1]),2);

                paths[i].cd = K_J * Jp + K_T * Ti + K_D * pow(paths[i].d[paths[i].limit-1], 2);
                paths[i].cv = K_J * Js + K_T * Ti + K_D * ds;
                paths[i].cf = K_LAT * paths[i].cd + K_LON * paths[i].cv;
                i++;
            }
            free(lat_qp);
        }
    }
    return i; // num of paths
}


void calc_global_paths(FrenetPath* fplist, Spline2D* csp, int nop)
{
    int i,j;
    float ix, iy, i_yaw, di, dx, dy,c;
    for(i=0; i<nop; i++)
    {
        for(j=0; j < fplist[i].limit; j++)
        {
            ix = csp->calc_position_x(csp, fplist[i].s[j]);
            iy = csp->calc_position_y(csp, fplist[i].s[j]);
			if (ix == -1)
			{
				fplist[i].limit = j-1;
				break;
			}
            i_yaw = csp->calc_yaw(csp, fplist[i].s[j]);
			di = fplist[i].d[j];
            fplist[i].x[j] = ix + di * cos(i_yaw + M_PI / 2.0);
            fplist[i].y[j] = iy + di * sin(i_yaw + M_PI / 2.0);
        }

        for(j=0; j < fplist[i].limit -1; j++)
        {
            dx = fplist[i].x[j+1] - fplist[i].x[j];
            dy = fplist[i].y[j+1] - fplist[i].y[j];
            fplist[i].yaw[j] = atan2(dy, dx);
            fplist[i].ds[j] = sqrt( dy*dy + dx*dx);
        }
        fplist[i].yaw[j] = fplist[i].yaw[j-1];
        fplist[i].ds[j] = fplist[i].ds[j-1];

        for(j=0; j < fplist[i].limit -1; j++)
        {
			c = (fplist[i].yaw[j + 1] - fplist[i].yaw[j]);
            if(c > M_PI) c -= 2*M_PI;
            else if(c < -M_PI) c += 2*M_PI;
			c = fabs(c) / fplist[i].ds[j];
			fplist[i].c[j] = c;
        }
    }
}

int check_paths(FrenetPath* fplist,int* ok_index, int nop) {
	int i, idx=0;
	float a;
	for (i = 0; i < nop; i++) {
		int ok = 1;
		for (int ii = 0; ii < fplist[i].limit; ii++)
			if (fplist[i].s_d[ii] > MAX_SPEED)
			{
				ok = 0;
				break;
			}
		if (!ok) continue;

		for (int ii = 0; ii < fplist[i].limit; ii++)
			if (fabs(fplist[i].s_dd[ii]) > MAX_ACCEL)
			{
				ok = 0;
				break;
			}
		if (!ok) continue;
		for (int ii = 0; ii < fplist[i].limit-1; ii++)
			if (fabs(fplist[i].c[ii]) > 0.5)
			{
				ok = 0;
				break;
			}
		if (!ok) continue;
		ok_index[idx++] = i;
	}
	return idx;
}

FrenetPath frenet_optimal_planning(Spline2D* csp, float s0, float c_speed, float c_d, float c_d_d, float c_d_dd){
    int nop; // number of paths
	int ok_index[MAX_PATH] = {0};
    FrenetPath bestPath;
    FrenetPath fplist[MAX_PATH];
	float min_cost = FLT_MAX;
	float cf = 0;

    nop = calc_frenet_paths(fplist, c_speed, c_d, c_d_d, c_d_dd, s0);
    calc_global_paths(fplist, csp, nop);
	nop = check_paths(fplist, ok_index, nop);

    for(int i=0;i<nop;i++)
        if (min_cost >= fplist[ok_index[i]].cf){
            min_cost = fplist[ok_index[i]].cf;
            bestPath = fplist[ok_index[i]];
        }
    
    return bestPath;
}

Spline2D* generate_target_course(float* x, float* y, int size, float* rrx, float* rry, float* rryaw, float* rrk){
	Spline2D* csp = init(x,y,size);
    float i_s = 0;
    int i=0;

    for(;i_s < csp->s[csp->size-1]; i_s += 0.1){
		rrx[i] = csp->calc_position_x(csp, i_s);
		rry[i] = csp->calc_position_y(csp, i_s);
        rryaw[i]=csp->calc_yaw(csp, i_s);
        rrk[i]=csp->calc_curvature(csp, i_s);
		i++;
    }
	return csp;
}

void rotate(float* nx, float* ny, float tx, float ty, float cx, float cy, float q)
{
	float cosq = cos(q), sinq = sin(q);

	// 회전중심점 C가 원점  O와 일치하도록 회전할 점 T를 함께 평행이동
	tx -= cx, ty -= cy;

	// 원점 O에 대하여 회전할 점 T를 q라디안 만큼 회전
	*nx = tx * cosq - ty * sinq;
	*ny = ty * cosq + tx * sinq;
}

void PathFuncCoef(Spline2D *csp, FrenetPath fp,float c_x, float c_y, float theta, float* co)
{
	float x[4], y[4];
	float tx, ty;
	for (int i = 0; i < 4; i++)
	{
		tx = csp->calc_position_x(csp, fp.s[0] + i);
		ty = csp->calc_position_y(csp, fp.s[0] + i);
		rotate(&(x[i]), &(y[i]), tx, ty, c_x, c_y, theta);
	}
	Spline* sp = init_spline(x,y,4);
	sp->GetCoef(sp, co);
	free(sp);
}

#define M_PI 3.14159265358979323846
using namespace std;
DataWarehouse dw;
PathMaker pm(&dw);
EventHandler eh(pm.lp, &dw);
ros::Publisher headingPub, spPub, pathPointPub, pathPointPub2, path_pub, marker4_pub;
ros::Publisher coordPubX;
ros::Publisher coordPubY;
ros::Publisher coordPubX2;
ros::Publisher coordPubY2;
ros::Publisher coordPubX3;
ros::Publisher coordPubY3;
ros::Publisher nearPubX;
ros::Publisher nearPubY;
ros::Publisher backPubX;
ros::Publisher backPubY;
void printPath(){
    visualization_msgs::MarkerArray markerArray;
    for(int i=0;i<10;i++){
            visualization_msgs::Marker marker;
            marker.header.frame_id = "/map";
            marker.ns="myPath";
            marker.id=i;
            marker.type = marker.SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = 0.5;
            marker.color.a = 1;
            marker.color.r = 1;
            marker.color.g = 0;
            marker.color.b = 0;
            marker.pose.orientation.w = 1.0;
            marker.lifetime = ros::Duration(10, 0);
            marker.pose.position.x = dw.globalCoordX+pm.gp->pathPointX[i];
            marker.pose.position.y = dw.globalCoordY+pm.gp->pathPointY[i];
            marker.pose.position.z = 0;
            markerArray.markers.push_back(marker);
        }
    pathPointPub.publish(markerArray);
}
void speed_callback(const std_msgs::UInt16::ConstPtr& msg){
    dw.velocity = msg->data;
}
float quatToDegree(float x, float y, float z, float w){
    float yaw = atan2(2.0*(y*x+z*w),(-z*z-y*y+x*x+w*w));
    yaw=yaw*180/3.1415;
    return yaw;
}
void lightCallback(const std_msgs::Int8::ConstPtr& msg){
    /*
        green light 1
        red light 2
        left light 3
    */
    cout<<"Traffic light"<<msg->data;
    switch(msg->data){
        case 1:
            cout<<"GREEN!!!";
            dw.light=DataWarehouse::GREEN;
            break;
        case 2:
            cout<<"RED!!!";
            dw.light=DataWarehouse::RED;
            break;
        case 3:
            cout<<"LEFT!!!";
            dw.light=DataWarehouse::LEFT;
            break;
        default:
            //dw.light==DataWarehouse::NULLLIGHT;
            break;
    }
}  
void velo_publish(){
    std_msgs::Float32 velo_msg;
    velo_msg.data = dw.maxVelo;
    spPub.publish(velo_msg);
}
void path_publish(){
    std_msgs::Float32 msgX;
    std_msgs::Float32 msgY;
    std_msgs::Float32 msgX2;
    std_msgs::Float32 msgY2;
    std_msgs::Float32 msgX3;
    std_msgs::Float32 msgY3;
    std_msgs::Float32 msgNearX;
    std_msgs::Float32 msgNearY;
    std_msgs::Float32 msgBackX;
    std_msgs::Float32 msgBackY;
    std_msgs::Float32 msgHeading;    
    
    msgX.data=pm.getCoordX();
    msgY.data=pm.getCoordY();
    msgX2.data=pm.getCoordX2();
    msgY2.data=pm.getCoordY2();
    msgX3.data=pm.getCoordX3();
    msgY3.data=pm.getCoordY3();
    msgNearX.data=pm.getNearCoordX();
    msgNearY.data=pm.getNearCoordY();
    msgBackX.data=pm.getBackCoordX();
    msgBackY.data=pm.getBackCoordY();
    msgHeading.data=dw.heading;

    coordPubX.publish(msgX);
    coordPubY.publish(msgY);
    coordPubX2.publish(msgX2);
    coordPubY2.publish(msgY2);
    coordPubX3.publish(msgX3);
    coordPubY3.publish(msgY3);
    nearPubX.publish(msgNearX);
    nearPubY.publish(msgNearY);
    backPubX.publish(msgBackX);
    backPubY.publish(msgBackY);
    headingPub.publish(msgHeading);
}
void odomCallback(nav_msgs::Odometry msg){
    float offX,offY;
    offX=offY=0;
        // filoty
        // offX=346208;
        // offY=4069632;
        //kcity
        // offX=302459.72;
        // offY=4123708.77;
        // 3gong
        // offX = 346391.9;
        // offY = 4070174.2;
        // pg
        offX = 346850.5;
        offY = 4069908.3;
        // narea
        // offX = 346768;
        // offY = 4070385; 
        // Sanhak
        // offX = 346243;
        // offY = 4069674; 
    // cout<<"---------------------ODOM COME-------------"<<endl;
    // cout<<"x = "<<msg.pose.pose.position.x-offX<<endl;
    // cout<<"y = "<<msg.pose.pose.position.y-offY<<endl;
    float x=msg.pose.pose.orientation.x;
    float y=msg.pose.pose.orientation.y;
    float z=msg.pose.pose.orientation.z;
    float w=msg.pose.pose.orientation.w;

    float yaw = quatToDegree(x,y,z,w);
    dw.heading=yaw;
    // cout<<"HEADING  = "<<dw.heading<<endl;
    float realX = 1.1*cos(yaw*M_PI/180);
    float realY = 1.1*sin(yaw*M_PI/180);

    realX=0;
    realY=0;

    // pm.gp->globalCoordX=msg.pose.pose.position.x;
    dw.globalCoordX = msg.pose.pose.position.x-offX+realX;

    // pm.gp->globalCoordY=msg.pose.pose.position.y;
    dw.globalCoordY = msg.pose.pose.position.y-offY+realY;
    
}
int ccw(float x1,float x2,float x3,float y1,float y2,float y3){
    float re = x1*y2 + x2*y3 + x3*y1 - (x2*y1 + x3*y2 + x1*y3);
    if(re<0)
    return -1;
    else if (re>0)
    return 1;
    else
    return 0;
}
void frenet(){

    float x[10];
    float y[10];
    float gx = dw.globalCoordX;
    float gy = dw.globalCoordY;
    for(int i=0;i<10;i++){
        x[i] = gx + pm.gp->pathPointX[i];
        y[i] = gy + pm.gp->pathPointY[i];
    }
    int ccwise = ccw(pm.gp->backCoordX, pm.gp->nearCoordX, 0, pm.gp->backCoordY, pm.gp->nearCoordY, 0);
    float dist = sqrt(pm.gp->nearCoordX*pm.gp->nearCoordX + pm.gp->nearCoordY*pm.gp->nearCoordY);
    cout<<"DIS = "<<ccwise*dist<<endl;
    float rx[4000];
    float ry[4000];
    float ryaw[4000];
    float rk[4000];
    Spline2D* csp;
    FrenetPath path;


    float c_speed = 10.0/3.6;
    float c_d = ccwise*dist;
    float c_d_d = 0.0;
    float c_d_dd = 0.0;
    float s0 = 0.0;


	csp = generate_target_course(x,y,8,rx,ry,ryaw,rk);
    path = frenet_optimal_planning(csp, s0,c_speed,c_d,c_d_d,c_d_dd);


    visualization_msgs::MarkerArray markerArray;
    for(int i=0;i<10;i++){
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/map";
        marker.ns="markerArray22222";
        marker.id=i;
        marker.type = marker.SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;
        marker.color.a = 1;
        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 1;
        marker.pose.orientation.w = 1.0;
        marker.lifetime = ros::Duration(10, 0);
        marker.pose.position.x = path.x[i];
        marker.pose.position.y = path.y[i];
        marker.pose.position.z = 0;
        markerArray.markers.push_back(marker);
    }

    pathPointPub2.publish(markerArray);

    float coco[4];
    PathFuncCoef(csp, path, gx, gy, 0, coco);
    cout<<"COCO "<<coco[0]<<" "<<coco[1]<<" "<<coco[2]<<" "<<coco[3]<<" "<<endl;

    float tarY = gx*gx*gx*coco[0] + gx*gx*coco[1]+gx*coco[2]+coco[3];

    float resi=dw.globalCoordY - tarY;
    cout<< "RESIDUAL -  "<<resi<<endl;
    std_msgs::Float64 coData[4];
    for(int i=0;i<4;i++) coData[i].data=coco[i];
    coef1.publish(coData[0]);
    coef2.publish(coData[1]);
    coef3.publish(coData[2]);
    coef4.publish(coData[3]);
}
void printMarkers(){
    visualization_msgs::MarkerArray mark4;
    for(int i=0;i<5;i++){
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/map";
        marker.id=i;
        marker.type = marker.SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.color.a = 1;
        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 0;
        marker.pose.orientation.w = 1.0;
        marker.lifetime = ros::Duration(10, 0);
        float posX,posY, size;
        posX=posY=0;
        size = 0;
        marker.ns="mark4";
        switch(i)
        {
        case 0:
            posX = dw.globalCoordX;
            posY = dw.globalCoordY;
            marker.color.r = 1.0;
            marker.color.g = 1;
            marker.color.b = 1.0;
            size = 0.5;
            break;
        case 1:
            posX = dw.globalCoordX+pm.getCoordX();
            posY = dw.globalCoordY+pm.getCoordY();
            break;
        case 2:
            posX = dw.globalCoordX+pm.getCoordX2();
            posY = dw.globalCoordY+pm.getCoordY2();
            break;
        case 3:
            posX = dw.globalCoordX+pm.getCoordX3();
            posY = dw.globalCoordY+pm.getCoordY3();
            break;
        case 4:
            posX = dw.globalCoordX+pm.getNearCoordX();
            posY = dw.globalCoordY+pm.getNearCoordY();
            marker.color.r = 0.0;
            marker.color.b = 1.0;
            break; 
        default:
            break;
        }
        marker.scale.x = size;
        marker.scale.y = size;
        marker.scale.z = size;
        marker.pose.position.x = posX;
        marker.pose.position.y = posY;
        marker.pose.position.z = 0;
        mark4.markers.push_back(marker);
    }
    marker4_pub.publish(mark4);
}
void printGlobalPath(){
    float size=0.3;
    visualization_msgs::MarkerArray markerArray;
    for(int i=0; i<pm.gp->pathQueueSize;i++){
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/map";
        marker.id=i;
        marker.ns="path";
        marker.type = marker.SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = size;
        marker.scale.y = size;
        marker.scale.z = size;
        marker.color.a = 1;
        marker.color.r = 0.0;
        marker.color.g = 0;
        marker.color.b = 1.0;
        marker.pose.orientation.w = 1.0;
        marker.pose.position.x = stof(pm.gp->pathX[i]);
        marker.pose.position.y = stof(pm.gp->pathY[i]);
        marker.pose.position.z = -0.5;
        markerArray.markers.push_back(marker);
    }
    path_pub.publish(markerArray);
}
int main(int argc, char** argv){
    
    //------------------ROS INIT-------------------------------------
    ros::init(argc, argv, "Control_pkg");
    ros::NodeHandle nh;
    
    //--------------------------MSG-------------------------------
    //-------------TO---------------------
    //------- To Control------
    //---COORD transmit-------
    coordPubX = nh.advertise<std_msgs::Float32>("pathX",100);
    coordPubY = nh.advertise<std_msgs::Float32>("pathY",100);
    coordPubX2 = nh.advertise<std_msgs::Float32>("pathX2",100);
    coordPubY2 = nh.advertise<std_msgs::Float32>("pathY2",100);
    coordPubX3 = nh.advertise<std_msgs::Float32>("pathX3",100);
    coordPubY3 = nh.advertise<std_msgs::Float32>("pathY3",100);
    nearPubX = nh.advertise<std_msgs::Float32>("nearX",100);
    nearPubY = nh.advertise<std_msgs::Float32>("nearY",100);
    backPubX = nh.advertise<std_msgs::Float32>("backX",100);
    backPubY = nh.advertise<std_msgs::Float32>("backY",100);
    pathPointPub = nh.advertise<visualization_msgs::MarkerArray>("/path_point",100);
    pathPointPub2 = nh.advertise<visualization_msgs::MarkerArray>("/path_point_fre",100);
    coef1 = nh.advertise<std_msgs::Float32>("/coef1",100);
    coef2 = nh.advertise<std_msgs::Float32>("/coef2",100);
    coef3 = nh.advertise<std_msgs::Float32>("/coef3",100);
    coef4 = nh.advertise<std_msgs::Float32>("/coef4",100);
    headingPub = nh.advertise<std_msgs::Float32>("heading2",100);

    //---Stop Line ---
    ros::Publisher stopBrakePub = nh.advertise<std_msgs::Float32>("/brake_sign",10);

    //------RVIZ---
    path_pub = nh.advertise<visualization_msgs::MarkerArray>("/path_rviz",100);
    marker4_pub = nh.advertise<visualization_msgs::MarkerArray>("/mark4",100);
    //---NO USE STANLEY Mission---

    //---StaticObs Mission---

    //-------------FROM---------------------
    //------- From 3DRiDAR---
    //------- From 2DRiDAR--
    //------- From LOCAL--
    ros::Subscriber coor_sub = nh.subscribe("/odom", 1, odomCallback);    
    //------- From IMU--------
    //------- From MISSIONS--------
    //--------From CAMERA-----------------
    ros::Subscriber trafficLight_sub = nh.subscribe("/lights",10,lightCallback);
    
    //-------- From ENCODER-----------------
    //-------- From ETC-----------------
    ros::Subscriber speed_sub= nh.subscribe("/speed_r", 10,speed_callback);


    //----------------------------------------------------------------

    ROS_INFO("--------------------SYSTEM ON-------------------");
    ROS_INFO("Build Ver - 12");
    ROS_INFO("[Function_ON]MAIN_CONTROLL_ON");
    pm.setMachine(eh.machine);
    eh.machine->start();
    ros::Rate loop(200);
    int globalPathRate=100;
    int count=0;
    //-----------------------------------REFACTORING NEEDED



    spPub = nh.advertise<std_msgs::Float32>("/velo_max",10);
    dw.setSpeed(12,5);
    
    while(ros::ok()){
        frenet();
        eh.listen_Data();
        pm.makePath();
        eh.machine->printCurMode();
        velo_publish();
        path_publish();
        system("clear");
        cout<<endl<<"-------------------------------------------------"<<endl<<endl;
        ros::spinOnce();
        loop.sleep();
        //-------------------------PRINT MARKER--------------------------
        printPath();
        printMarkers();
        if(count>globalPathRate){
            printGlobalPath();
            count=0;
        }
        count++;
    }
    
    cout<<"Module Finished"<<endl;
    ros::shutdown();
    return 0;
}