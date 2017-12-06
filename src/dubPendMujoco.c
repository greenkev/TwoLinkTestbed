#include <stdio.h>
#include <stdbool.h>
#include "mujoco.h"
#include "dubPendMujoco.h"
#include <stddef.h>
#include <stdlib.h>
#include <math.h>
#include "stdlib.h"
#include "string.h"


static bool m_bMJActivated = false;
//static bool glfw_initialized = false;


unsigned char* m_image_rgb;
float* m_image_depth;

int m_Width;
int m_Height;
bool m_bSaveVideo;

int targetSiteIds[XDD_TARGETS];

FILE *fp;

struct dubpend_t {
	mjData *d;
};

/*struct dubpend_vis_t {
	GLFWwindow* window;
	mjvCamera cam;
	mjvOption opt;
	mjvScene scn;
	mjrContext con;
};*/

static mjModel* m;
static mjData* d;
static mjData* d_old; //container for moving data struct

void init(const char *basedir)
{

	if (!m_bMJActivated) {
        // If no base directory is provided, use . as the base directory
		const char thisdir[] = ".";
		if (!basedir)
			basedir = thisdir;
		size_t basedirlen = strnlen(basedir, 4096);

		// Construct key file path
		char *keyfile;
		const char keyfilename[] = "mjkey.txt";
		keyfile = malloc(basedirlen + 1 + sizeof keyfilename);
		strncpy(keyfile, basedir, basedirlen);
		keyfile[basedirlen] = '/';
		strcpy(keyfile + basedirlen + 1, keyfilename);// Construct XML file path
		char *xmlfile;
		const char xmlfilename[] = "dubPend.xml";
		xmlfile = malloc(basedirlen + 1 + sizeof xmlfilename);
		strncpy(xmlfile, basedir, basedirlen);
		xmlfile[basedirlen] = '/';
		strcpy(xmlfile + basedirlen + 1, xmlfilename);

		// Activate mujoco and load the model
		mj_activate(keyfile);
		char error[1000] = "Could not load XML model";
		m = mj_loadXML(xmlfile, 0, error, 1000);
		if (m) {
			m_bMJActivated = true;
		} else {
			mju_error_s("Load model error: %s", error);
		}

		// Free allocated strings
		free(keyfile);
		free(xmlfile);
    }
	m_bMJActivated = true;

    d = mj_makeData(m);
    d_old = mj_makeData(m);

    double qpos_init[] = { 0,0};

//    		-0.00004955, -0.00000001, 1.00803840, 0.99999991, 0.00000009, -0.00042946, 0.00000031,
//    -0.03050475, -0.00000066, 0.49730789, -1.19993806, -0.00000000, 1.42138220, -1.59674847,
//    -1.52494131, 0.64763779, 0.00000000, 0.97865137, -0.01640312, 0.01779799, -0.20409722,
//    -0.03050472,-0.00000065, 0.49730777, -1.19993824, -0.00000000 , 1.42138238 , -1.59674847 ,
//    -1.52494131 , 0.64763779 , 0.00000000 , 0.97878119 , 0.00386090 , -0.01518115 , -0.20430861 };

	mju_copy(d->qpos, qpos_init, m->nq);

    mj_forward(m, d);

    targetSiteIds[0] = mj_name2id(m, mjOBJ_SITE, "end_effector");

}

void set_state(double* q, double* qd)
{
	mju_copy(d->qpos, q, m->nq);
	mju_copy(d->qvel, qd, m->nv);
	mju_zero(d->qacc, m->nv);
	mju_zero(d->qacc_warmstart, m->nv);
	mj_forward(m, d);
}

void get_dynamic_info(dyn_info_t_muj* info)
{
	//mass matrix
	mjtNum M[nQ*nQ];
	mj_fullM(m, M, d->qM);
	for (int i = 0; i < nQ*nQ; i++)
		info->H[i] = M[i];

	//Cqdot + G - tau_passive
	for (int i = 0; i < nQ; i++)
	{
		info->qfrc_bias[i] = d->qfrc_bias[i]; //This is real sketchy
		info->qfrc_passive[i] = d->qfrc_passive[i];
	}

	//selector matrix
	for (int i = 0; i < nQ*nU; i++)
		info->B[i] = d->actuator_moment[i];

	mjtNum jacr[nQ*3]; //rotation
	mjtNum jacp[nQ*3]; //position
	for (int i = 0; i < XDD_TARGETS; i++)
	{
		mj_jacSite(m, d, jacp, jacr, targetSiteIds[i]);

		for (int j = 0; j < DOF; j++)
			for (int k = 0; k < nQ; k++)
				info->A[i*(3+DOF)*nQ + j*nQ + k] = jacp[j*nQ + k];
		for (int j = 0; j < DOF; j++)
			for (int k = 0; k < nQ; k++)
				info->A[i*(3+DOF)*nQ + (3+j)*nQ + k] = jacr[j*nQ + k];
	}
}

void step(double* u)
{
	for (int i = 0; i < m->nu; i++)
		d->ctrl[i] = u[i];
	mj_step(m, d);
}

/*void get_com_pos(double* com_pos)
{
	mju_zero(com_pos,3);
	mjtNum totalMass = mj_getTotalmass(m);
	for (int i = 1; i < m->nbody; i++)
	{
		mju_addToScl3(com_pos, &(d->xipos[i*3]), m->body_mass[i]/totalMass);
	}
}*/

/*void get_joint_limits(pos_limits_t *lim)
{
	for (int i = 0; i < nQ; i++)
	{
		if (!m->jnt_limited[i])
		{
			lim->lb[i] = -1e20;
			lim->ub[i] = 1e20;
		}
		else {
			lim->lb[i] = m->jnt_range[i*2];
			lim->ub[i] = m->jnt_range[i*2+1];
		}
	}
}*/

/*void get_motor_limits(motor_limits_t *lim)
{
	for (int i = 0; i < nU; i++)
	{
		if (!m->actuator_ctrllimited[i])
		{
			lim->lb[i] = -1e20;
			lim->ub[i] = 1e20;
		}
		else {
			lim->lb[i] = m->actuator_ctrlrange[i*2];
			lim->ub[i] = m->actuator_ctrlrange[i*2+1];
		}
	}
}*/

void get_state(state_t_muj* s)
{
	mju_copy(s->q, d->qpos, m->nq);
	mju_copy(s->qd, d->qvel, m->nv);
	mju_copy(s->qdd, d->qacc, m->nv);
	mjtNum cvel[6];
	mjtNum cacc[6];
	for (int i = 0; i < XDD_TARGETS; i++)
	{
		int site_id = targetSiteIds[i];

		mju_copy(&(s->xpos[i*3]), &(d->site_xpos[site_id*3]), 3);

		mj_objectVelocity(m, d, mjOBJ_SITE, site_id, cvel, 0);
		mju_copy(&(s->xvel[i*6]), cvel, 6);
		mj_objectAcceleration(m, d, mjOBJ_SITE, site_id, cacc, 0);
		mju_copy(&(s->xacc[i*6]), cacc, 6);
	}
}


/*static void window_close_callback(GLFWwindow* window)
{
    vis_close(glfwGetWindowUserPointer(window));
}

static void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
	Scroll(xoffset, yoffset, glfwGetWindowUserPointer(window));
}
static void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
	MouseMove(xpos, ypos, glfwGetWindowUserPointer(window));
}
static void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
	MouseButton(button, act, mods, glfwGetWindowUserPointer(window));
}

static void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
	Keyboard(key, scancode, act, mods, glfwGetWindowUserPointer(window));
}*/


/*cassie_vis_t* vis_init(bool save_video)
{
    // Check if model has been loaded
    if (!m) {
        mju_error("init must be called before vis_init");
        return NULL;
    }

    // Initialize GLFW if this is the first visualization window
    if (!glfw_initialized) {
        if (!glfwInit()) {
            mju_error("Could not initialize GLFW");
            return NULL;
        }
        glfw_initialized = true;
    }

    // Allocate visualization structure
    cassie_vis_t *v = malloc(sizeof (cassie_vis_t));

    m_Width = 1200;
	m_Height = 900;

    // Create window
    v->window = glfwCreateWindow(m_Width, m_Height, "Slip", NULL, NULL);
    glfwMakeContextCurrent(v->window);
    glfwSwapInterval(1);

    // Set up mujoco visualization objects
    v->cam.lookat[0] = m->stat.center[0];
	v->cam.lookat[1] = m->stat.center[1];
	v->cam.lookat[2] = 0.8 + m->stat.center[2];
	v->cam.type = mjCAMERA_FREE;

	v->cam.distance = 0.5 * m->stat.extent;
	mjv_moveCamera(m, mjMOUSE_ROTATE_H, 0.75, 0.0, &v->scn, &v->cam);

    mjv_defaultOption(&v->opt);
    mjr_defaultContext(&v->con);
    mjv_makeScene(&v->scn, 1000);
    mjr_makeContext(m, &v->con, mjFONTSCALE_100);

    // Set callback for user-initiated window close events
    glfwSetWindowUserPointer(v->window, v);
    glfwSetWindowCloseCallback(v->window, window_close_callback);
	glfwSetCursorPosCallback(v->window, mouse_move);
	glfwSetMouseButtonCallback(v->window, mouse_button);
	glfwSetScrollCallback(v->window, scroll);
	glfwSetKeyCallback(v->window, keyboard);

	if (save_video)
	{
		m_image_rgb = (unsigned char*)malloc(3*m_Width*m_Height);
		m_image_depth = (float*)malloc(sizeof(float)*m_Width*m_Height);

		// create output rgb file
		fp = fopen("out/temp.out", "wb");
		if( !fp )
			mju_error("Could not open rgbfile for writing");
	}

	m_bSaveVideo = save_video;

    return v;
}

static bool bUserInput = false;

bool vis_draw(cassie_vis_t *v, bool bWaitUser)
{
	bool doOnce = true;

	mjrRect viewport = {0, 0, 0, 0};

	while (bWaitUser || doOnce)
	{
		doOnce = false;
		// Return early if window is closed
		if (!v->window)
			return false;

		// Set up for rendering
		glfwMakeContextCurrent(v->window);

		glfwGetFramebufferSize(v->window, &viewport.width, &viewport.height);

		// Render scene
		mjv_updateScene(m, d, &v->opt, NULL, &v->cam, mjCAT_ALL, &v->scn);
		mjr_render(viewport, &v->scn, &v->con);

		// Show updated scene
		glfwSwapBuffers(v->window);
		glfwPollEvents();

		if (bUserInput)
		{
			bUserInput = false;
			break;
		}

	}

	if (m_bSaveVideo)
	{
		mjr_readPixels(m_image_rgb, m_image_depth, viewport, &v->con);
		// insert subsampled depth image in lower-left corner of rgb image
//		const int NS = 3;           // depth image sub-sampling
//		for( int r=0; r<m_Height; r+=NS )
//		   for( int c=0; c<m_Width; c+=NS )
//		   {
//			  int adr = (r/NS)*m_Width + c/NS;
//			  m_image_rgb[3*adr] = m_image_rgb[3*adr+1] = m_image_rgb[3*adr+2] = (unsigned char)((1.0f-m_image_depth[r*m_Width+c])*255.0f);
//		   }

		 // write rgb image to file
		 fwrite(m_image_rgb, 3, m_Width*m_Height, fp);
	}

	return true;
}

static bool button_left = false;
static bool button_middle = false;
static bool button_right =  false;
static double cursor_lastx = 0;
static double cursor_lasty = 0;


// mouse button
void MouseButton(int button, int act, int mods, cassie_vis_t* v)
{
    // update button state
    button_left =   (glfwGetMouseButton(v->window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(v->window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(v->window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(v->window, &cursor_lastx, &cursor_lasty);
}



// mouse move
void MouseMove(double xpos, double ypos, cassie_vis_t* v)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - cursor_lastx;
    double dy = ypos - cursor_lasty;
    cursor_lastx = xpos;
    cursor_lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(v->window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(v->window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(v->window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    mjv_moveCamera(m, action, dx/height, dy/height, &v->scn, &v->cam);
}

void Scroll(double xoffset, double yoffset, cassie_vis_t* v)
{
    // scroll: emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &v->scn, &v->cam);
}

// keyboard
void Keyboard(int key, int scancode, int act, int mods, cassie_vis_t* v)
{
    // do not act on release
    if( act==GLFW_RELEASE )
        return;

    bUserInput = true;
}

void vis_close(cassie_vis_t *v)
{
    if (!v || !v->window)
        return;

    // Free mujoco objects
    mjv_freeScene(&v->scn);
    mjr_freeContext(&v->con);

    // Close window
    glfwDestroyWindow(v->window);
    v->window = NULL;

}

*/