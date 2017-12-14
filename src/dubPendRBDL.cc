/* dubPendRBDL.cc
* Created by Kevin Green on Dec 7, 2017
* Copyright 2017 - Under creative commons license 3.0:
*        Attribution-ShareAlike CC BY-SA
*
* This software is furnished "as is", without technical support, and with no 
* warranty, express or implied, as to its usefulness for any purpose.
*
* This is a C++ wrapper with a C interface for a simple RBDL double pendulum model.
* This is intended to be used as a part of a controller, while MuJoCo is intended
* to be the simulation enviroment. This seperation is to allow the simulator to be
* replaced with a physical system with minimal interface changes.
* It is designed to be compiled into a library and loaded into matlab 
* through a matlab class. It uses the state_t structure and the dynamic_info_t
* structure to return data. It requires RBDL and Eigen3. 
*
* @author Kevin Green - greenkev@oregonstate.edu
*/
#include <rbdl/rbdl.h>
#include "dubPendRBDL.h"

#define DT 0.0005

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

static Model* m;
static state_t* st;

unsigned int endEffBodyID;
unsigned int body_a_id, body_b_id;
static Vector3d endEffPtPos;

void init(){

  	//rbdl_check_api_version (RBDL_API_VERSION);

	//build model
	m = new Model();
  	m->gravity = Vector3d (0., 0., -9.81);

  	//Body and joint declarations
	Body body_a, body_b;
	Joint joint_a, joint_b;

	//See RBDL documentation for more info on the model generation syntax
	endEffBodyID = 0;

	body_a = Body (1., Vector3d (0.5, 0., 0.0), Vector3d (0.1,0.1,0.1));
	joint_a = Joint(JointTypeRevolute, Vector3d (0., 0., 1.));

	body_a_id = m->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_a, body_a);

	body_b = Body (1., Vector3d (0.5, 0., 0.0), Vector3d (0.1,0.1,0.1));
	joint_b = Joint(JointTypeRevolute, Vector3d (0., 0., 1.));

	body_b_id = m->AddBody(body_a_id, Xtrans(Vector3d(1, 0., 0.)), joint_b, body_b);


	endEffBodyID = body_b_id;
	endEffPtPos = Vector3d(1., 0., 0.);

	st = new state_t(); 
}

void get_dynamic_info(dyn_info_t* info){
	//Copy State Vectors
	VectorNd Q = VectorNd::Zero (m->dof_count);
	VectorNd QDot = VectorNd::Zero (m->dof_count);
	VectorNd QDDot = VectorNd::Zero (m->dof_count);
	for(int i = 0; i < nQ; ++i){
		Q[i] = st->q[i];
		QDot[i] = st->qd[i];
	}

	//Mass Matrix
	MatrixNd H = MatrixNd::Zero(nQ,nQ);
	CompositeRigidBodyAlgorithm(*m, Q, H);
	for(int i = 0; i < nQ; ++i){
		for(int j = 0; j < nQ; ++j){
			info->H[i*nQ+j] = H(i,j);
		}
	}

	// Generalized Joint Forces (grav, coriolis, ect.)
	// Calculated two ways for comparison
	VectorNd h = VectorNd::Zero (m->dof_count);	
	NonlinearEffects(*m, Q, QDot, h);
	for(int i = 0; i < nQ; ++i){
		info->h[i] = h[i];
	}

	//End Eff Jacobian
	MatrixNd Jc = MatrixNd::Zero(6,nQ);
	CalcPointJacobian6D(*m, Q, endEffBodyID, endEffPtPos, Jc);
	for(int i = 0; i < 6; ++i){
		for(int j = 0; j < nQ; ++j){
			info->Jc[i*nQ + j] = Jc(i,j);
		}
	}

	//Finite Difference Jc to get JcDot
	MatrixNd JcNext = MatrixNd::Zero(6,nQ);
	CalcPointJacobian6D(*m, Q + QDot*DT, endEffBodyID, endEffPtPos, JcNext);
	for(int i = 0; i < 6; ++i){
		for(int j = 0; j < nQ; ++j){
			info->JcDot[i*nQ + j] = (JcNext(i,j) - Jc(i,j))/DT;
		}
	}
}

void set_state(double *q, double *qd){

	VectorNd Q = VectorNd::Zero (m->dof_count);
	VectorNd QDot = VectorNd::Zero (m->dof_count);

	for(int i = 0; i < nQ; ++i){
		st->q[i] = q[i];
		st->qd[i] = qd[i];
		Q[i] = st->q[i];
		QDot[i] = st->qd[i];
	}

	Vector3d endEffLoc;
	//Math::Vector3d endEffRot;
	SpatialVector endEffVel;

	endEffLoc = CalcBodyToBaseCoordinates(*m, Q, endEffBodyID, endEffPtPos);
	//endEffRot = CalcBodyWorldOrientation(*m, Q, endEffBodyID);
	endEffVel = CalcPointVelocity6D(*m, Q, QDot, endEffBodyID, endEffPtPos);

	for(int i = 0; i < DOF; ++i){
		st->xpos[i] = endEffLoc(i);
	}
	for(int i = 0; i < 2*DOF; ++i){
		st->xvel[i] = endEffVel(i);
	}
	
}

void get_state(state_t* s){

	//Copy all from st to s
	for(int i = 0; i < nQ; ++i){
		s->q[i] = st->q[i];
		s->qd[i] = st->qd[i];
	}

	for(int i = 0; i < DOF; ++i){
		s->xpos[i] = st->xpos[i];
	}

	for(int i = 0; i < 2*DOF; ++i){
		s->xvel[i] = st->xvel[i];
	}
}

/*void step(double* u){

}*/
