
#include <windows.h>
#include "GL/glew.h"
#include "GL/gl.h"			
#include "GL/glu.h"
#include "Cg/cg.h"
#include "Cg/cgGL.h"

#include <stdio.h>
#include <math.h>
#include <fcntl.h>
#include <io.h>
#include <iostream>
#include <fstream>
using namespace std;

#ifdef __cplusplus
extern "C"
{
#endif

#include "math.h"
#include "physics.h"
#include "rendering.h"
#include "collision_detection.h"
#include "isosurface.h"

#ifdef __cplusplus
}
#endif

//#pragma float_control(precise, on)
//#pragma fenv_access(on)
//#pragma float_control(except, on)
//#pragma fp_contract(off)

#define WINDOW_WIDTH		800
#define WINDOW_HEIGHT		600
#define FRUSTRUM_ANGLE		90.0f
#define FRUSTRUM_NEAR		0.3f
#define FRUSTRUM_FAR		200.0f

#define MAX_CONSOLE_LINES	500

// window's stuff
static HINSTANCE	hInstance;
static HDC			hDC;
static HWND			hWnd;
static bool			fullscreen;
static float		w_width, w_height;

/* sph */
static sph_property		properties;
static sph_machine		machine;
static iso_data			data;
static int				pause;

static float			sphere_radius;
static point3f			sphere_center;
static vector3f			gravity;

/* transformation */
static float			tb_radius;
static int				tb_active;
static quat4f			last_q_rotation, curr_q_rotation;
static quat4f			last_q_ball_rotation, curr_q_ball_rotation;
static matrix44f		m_rotation, m_ball_rotation;

/* mouse's stuff */
static point3f			last_coor, curr_coor;
static int				shift_down;

/* cg */
static CGcontext	cg_context;
static CGprofile	cg_vprof, cg_fprof;
static CGprogram	cg_vp, cg_fp;
static CGparameter	cg_fprm_bump_tex, cg_fprm_bk_tex
					, cg_vprm_mvp;
static const char	*vp_filename = "shaders/vp.cg",
					*fp_filename = "shaders/fp.cg",
					*cg_func_name = "main";
/* skybox */
GLuint skybox;
static const char	*skybox_filname[6] = {"skybox/posx.bmp", "skybox/negx.bmp",
										"skybox/posy.bmp", "skybox/negy.bmp",
										"skybox/posz.bmp", "skybox/negz.bmp"};
static const float cube_coords[24][3] = {
    /* positive x face. */
    {1.0f, -1.0f, -1.0f}, {1.0f, -1.0f, 1.0f}, {1.0f, 1.0f, 1.0f}, {1.0f, 1.0f, -1.0f},
    /* negative x face. */
    {-1.0f, -1.0f, 1.0f}, {-1.0f, -1.0f, -1.0f}, {-1.0f, 1.0f, -1.0f}, {-1.0f, 1.0f, 1.0f},
    /* positive y face. */
    {-1.0f, 1.0f, -1.0f}, {1.0f, 1.0f, -1.0f}, {1.0f, 1.0f, 1.0f}, {-1.0f, 1.0f, 1.0f},
    /* negative y face. */
    {-1.0f, -1.0f, 1.0f}, {1.0f, -1.0f, 1.0f}, {1.0f, -1.0f, -1.0f}, {-1.0f, -1.0f, -1.0f},
    /* positive z face. */
    {1.0f, -1.0f, 1.0f}, {-1.0f, -1.0f, 1.0f}, {-1.0f, 1.0f, 1.0f}, {1.0f, 1.0f, 1.0f},
    /* negatieve z face. */
    {-1.0f, -1.0f, -1.0f}, {1.0f, -1.0f, -1.0f}, {1.0f, 1.0f, -1.0f}, {-1.0f, 1.0f, -1.0f}
  };

/* frame rate */
static int frame_count = 0;
static float fps = 0.0f;