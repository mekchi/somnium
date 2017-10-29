
#ifndef _MEKCHI_RENDERING_
#define _MEKCHI_RENDERING_

#include <windows.h>
#include <GL/gl.h>			
#include <GL/glu.h>
//#include <GL/glut>

#include "common.h"
#include "isosurface.h"


typedef struct 
{
	point3f point[3];
} triangle;

typedef struct 
{
	point3f		point[8];
	vector3f	normal[8];
	float		value[8];
} mc_grid_cell;


void r_draw_points(iso_data *data);
void r_draw_marching_cubes(const iso_data *data, float threshold);



#endif

