
#ifndef _MEKCHI_COMMON_
#define _MEKCHI_COMMON_

#include <stdlib.h>

#ifdef _DEBUG

#include <stdio.h>
#include <assert.h>
#include <crtdbg.h>

#endif

#include "math.h"

float		*sph_f_link;
int			*sph_i_link;
vector3f	*sph_v_link;

point3f		*g_position;
vector3f	response;


int			iteration;
float		max_density;

/* particle information */
//point3f		cp; // current position
//vector3f	cv; // current velocity
//vector3f	cr; // current response

#define _DEFAULT
//#define _TEST_1

#define NUMBER_PARAMETERS	12
#define SIZE_INT_POOL		((2 + (max_neighbours * number)) * SIZE_INT)
#define SIZE_FLOAT_POOL		(((3 * NUMBER_PARTICLES)+ NUMBER_PARAMETERS + (MAX_NEIGHBOURS * NUMBER_PARTICLES)) * SIZE_FLOAT)
#define SIZE_VPC3F_POOL		((1 + (6 * NUMBER_PARTICLES)) * SIZE_VPC3F)

#define	NUMBER_PARTICLES	1000//sph_i_link[0]
#define MAX_NEIGHBOURS		25//sph_i_link[1]
#define POOL_INDEX			&sph_i_link[2]

#define H				0.01f//sph_f_link[0]
#define REST_DENSITY	1000.0f//sph_f_link[1]
#define STIFFNESS		1.5f//sph_f_link[2]
#define RESTITUTION		0.1f//sph_f_link[3]
#define VISCOSITY		0.2f//sph_f_link[4]
#define TIMESTEP		0.0035f//0.003f//sph_f_link[5]
#define P_MASS			0.00020543f//sph_f_link[6]
#define W_POLY6			(315.0f / (64.0f * PI * powf(H, 9.0f)))//sph_f_link[7]
#define W_SPIKY			(-45.0f / (PI * powf(H, 6.0f)))//sph_f_link[8]
#define W_DEFAULT		(-945.0f /(32.0f * PI * powf(H, 9.0f)))//sph_f_link[9]
#define W_VISCOSITY		(45.0f / (PI * powf(H, 6.0f)))//sph_f_link[10]
#define H_H				(H * H)//sph_f_link[11]

#define	DENSITY			&sph_f_link[NUMBER_PARAMETERS]
#define PRESSURE		&sph_f_link[NUMBER_PARAMETERS + NUMBER_PARTICLES]
#define MASS			&sph_f_link[NUMBER_PARAMETERS + (2 * NUMBER_PARTICLES)]
#define POOL_DISTANCE	&sph_f_link[NUMBER_PARAMETERS + (3 * NUMBER_PARTICLES)]

#define GRAVITY			&sph_v_link[0]
#define POSITION		&sph_v_link[1]
#define VELOCITY		&sph_v_link[1 + NUMBER_PARTICLES]
#define ACCELERATION	&sph_v_link[1 + (2 * NUMBER_PARTICLES)]
#define HALF_VElOCITY	&sph_v_link[1 + (3 * NUMBER_PARTICLES)]
#define F_PRESSURE		&sph_v_link[1 + (4 * NUMBER_PARTICLES)]
#define F_VISCOSITY		&sph_v_link[1 + (5 * NUMBER_PARTICLES)]

#define ISOVALUE		(P_MASS * W_POLY6 * H_H * H_H * H_H)
#define EPSILON			0.00001f

#ifdef _DEBUG

FILE *log_file;

void common_create_log_file();
void common_destroy_log_file();

#endif

#endif