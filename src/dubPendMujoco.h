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
// typedef struct dubpend_vis_t dubpend_vis_t;

void init(const char *basedir);
void set_state(double* q, double* qd);
void get_dynamic_info(dyn_info_t_muj* info);
void step(double* u);
//void get_com_pos(double* com_pose);
//void get_joint_limits(pos_limits_t *lim);
//void get_motor_limits(motor_limits_t *lim);
void get_state(state_t_muj* s);


/*cassie_vis_t *vis_init(bool bSaveVid);
bool vis_draw(cassie_vis_t *v, bool bWaitUser);
void vis_close(cassie_vis_t *v);
void Keyboard(int key, int scancode, int act, int mods, cassie_vis_t* v);
void Scroll(double xoffset, double yoffset, cassie_vis_t* v);
void MouseMove(double xpos, double ypos, cassie_vis_t* v);
void MouseButton(int button, int act, int mods, cassie_vis_t* v);*/


#endif  // dubpendmujoco_h__

