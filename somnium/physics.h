
#ifndef _MEKCHI_SPH_PHYSICS_
#define _MEKCHI_SPH_PHYSICS_

#include <float.h>

#include "common.h"
#include "spatial_partitioning.h"
#include "collision_detection.h"

//#define _BFNN
#define _RGNN
//#define _HTNN

#define NUMBER_CUBE_PARTS 26

typedef struct
{
	float		smooth_length;
	float		rest_density;
	float		stiffness;
	float		restitution;
	float		viscosity;
	float		timestep;
	float		mass;
} sph_property;

typedef struct
{
	float		*float_memory_pool;
	int			*int_memory_pool;
	vector3f	*vpc3f_memory_pool;
	int			*tag;
	//float		*density;
	/* particle */ 
	/*int			number_particles;
	
	float		*pressure;
	float		*mass;
	vector3f	*velocity;
	point3f		*position;
	vector3f	*acceleration;

	float		smooth_length;
	float		sq_smooth_length;*/
	
	/* spatial partitioning */
#ifdef _HTNN
	hash_table	table;
#endif
#ifdef _RGNN
	regular_grid	grid;
#endif
	
	/* nearest neighbour */
	int			*number_neighbours;
	int			**neighbours;
	float		**distances;
	point3f		cube[NUMBER_CUBE_PARTS];

	//int			*tns; // temporary buffer (indices)
	//float		*tds; // temporary buffer (distances)

	///* auxiliary variable */
	//float w_poly6;
	//float w_spiky;
	//float w_default;
	//float w_viscosity;

	//float		c_rest_dencity;
	//float		c_stiffness;
	//float		c_restitution;
	//vector3f	c_gravity;

	/* collision detection and response */
	cd_sphere sphere;

} sph_machine;

void sph_create(sph_machine *machine, int number, sph_property *params, int max_neighbours, int grid_size);
void sph_destroy(sph_machine *machine);
void sph_set_gravity(vector3f *g);
void sph_change_position(matrix44f rotation);
void sph_set_position(float interval, int xn, int yn, int zn);
void sph_set_collision_object(point3f *center, float radius);
void sph_set_current_sph(sph_machine *machine);
void sph_update();

#endif