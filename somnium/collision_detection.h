
#ifndef _MEKCHI_CD_
#define _MEKCHI_CD_

#include "common.h"
#include "math.h"

typedef struct
{
	vector3f normal;
	//point3f cp; // collision point
	float	depth; // penetration depth
} cd_info;

typedef struct
{
	point3f center;
	float size;
} obb;

typedef struct
{
	point3f center;
	float radius;
} cd_sphere;

int	cd_point_sphere(vector3f *velocity, point3f *new_position, point3f *position, cd_sphere *sphere);
//void	cd_point_obb(point3f *point, obb *object);


#endif
