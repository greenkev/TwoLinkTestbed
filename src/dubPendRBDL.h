#ifndef dubPendRBDL_h__
#define dubPendRBDL_h__

#include <stdbool.h>


#define nQ 2 //Dim of joint space
#define DOF 3 //Only consider 2-D constraints
#define nU 2 //Hip, knee, toe
#define nC 1 //One End Effector

#define XDD_TARGETS 1 //Single End Effector

#ifdef __cplusplus
extern "C" {
#endif

typedef struct state_t {
	double q[nQ];
	double qd[nQ];
	double xpos[DOF];//contact pos
	//double xOrient[DOF*DOF];//End Effector Rotation Matrix
	double xvel[DOF*2];//contact vel (rotational vel then position)
} state_t;

typedef struct dyn_info_t {
	double H[nQ*nQ];
	double h[nQ];
	double Jc[nQ*DOF*2];
	double JcDot[nQ*DOF*2];
} dyn_info_t;


void init();
void get_dynamic_info(dyn_info_t* info);
void set_state(double* q, double* qd);
void get_state(state_t* s);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif //dubPendRBDL_h__