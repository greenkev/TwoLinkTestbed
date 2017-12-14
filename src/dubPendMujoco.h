/* dubPendMuJoCo.cc
* Created by Kevin Green on Dec 7, 2017
* Copyright 2017 - Under creative commons license 3.0:
*        Attribution-ShareAlike CC BY-SA
*
* This software is furnished "as is", without technical support, and with no 
* warranty, express or implied, as to its usefulness for any purpose.
*
* This is a C++ wrapper with a C interface for a simple MuJoCo double pendulum model.
* This is intended to be the main forward dynamics simulator, while RBDL is intended
* to be a part of the controller. This seperation is to allow the simulator to be
* replaced with a physical system with minimal interface changes.
* This wrapper is designed to be compiled into a library and loaded into matlab 
* through a dedicated matlab class. It uses the state_t_muj, dyn_info_t_muj,
*  pos_limits_t, motor_limits_t and dubpend_t structure to return data.
* It requires RBDL and Eigen3. 
*
* @author Kevin Green - greenkev@oregonstate.edu
*/
#ifndef dubpendmujoco_h__
#define dubpendmujoco_h__
 

#include <stdbool.h>

#define nQ 2 //10 one-DOF joints, including the hip vertical slider 
//full = 32

#define nU 2 //Hip, knee, toe
#define DOF 3 //Only consider 2-D constraints

#define XDD_TARGETS 1 //end_effector only
#define MAX_NE_CONST 100

typedef struct state_t_muj {
	double q[nQ]; //extra for free joint quaternion
	double qd[nQ];
	double qdd[nQ];
	double xpos[XDD_TARGETS*DOF];//contact pos
	double xvel[XDD_TARGETS*DOF*2];//contact vel
	double xacc[XDD_TARGETS*DOF*2];//contact acc
} state_t_muj;

typedef struct dyn_info_t_muj {
	double A[XDD_TARGETS*nQ*DOF*2];
	double B[nQ*nU];
	double H[nQ*nQ];
	double qfrc_bias[nQ];
	double qfrc_passive[nQ];
} dyn_info_t_muj;

typedef struct pos_limits_t {
	double lb[nQ];
	double ub[nQ];
} pos_limits_t;

typedef struct motor_limits_t {
	double lb[nU];
	double ub[nU];
} motor_limits_t;

typedef struct dubpend_t dubpend_t;

void init(const char *basedir);
void set_state(double* q, double* qd);
void get_dynamic_info(dyn_info_t_muj* info);
void step(double* u);
//void get_com_pos(double* com_pose);
//void get_joint_limits(pos_limits_t *lim);
//void get_motor_limits(motor_limits_t *lim);
void get_state(state_t_muj* s);


#endif  // dubpendmujoco_h__

