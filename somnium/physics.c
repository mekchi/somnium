
#include "physics.h"

static sph_machine	*gs_sph;

static cd_sphere	*gs_sphere = NULL;
static point3f		*gs_cube = NULL;
static float		**gs_distances = NULL;
static int			**gs_neighbours = NULL;
static int			*gs_number_neighbours = NULL;

static float		*gs_mass = NULL;
static vector3f		*gs_velocity = NULL;
static vector3f		*gs_half_velocity = NULL;
static float		*gs_pressure = NULL;
static float		*gs_density = NULL;
static vector3f		*gs_acceleration = NULL;
static vector3f		*gs_f_pressure = NULL;
static vector3f		*gs_f_viscosity = NULL;

static float		*gs_temp_distance = NULL;
static int			*gs_temp_index = NULL;

static vector3f		gravity;
static int total;

void set_cube(sph_machine *machine)
{
	float h = H;
	/*float cube[3 * NUMBER_CUBE_PARTS] = {
		0.0f, 0.0f, h, 0.0f, 0.0f, -h, 0.0f, h, 0.0f
		, 0.0f, -h, 0.0f, 0.0f, h, h, 0.0f, h, -h
		, 0.0f, -h, h, 0.0f, -h, -h, h, 0.0f, h
		, h, 0.0f, -h, h, h, 0.0f, h, -h, 0.0f
		, h, h, h, h, h, -h, h, -h, h
		, h, -h, -h, h, 0.0f, 0.0f, -h, 0.0f, h
		, -h, 0.0f, -h, -h, h, 0.0f, -h, -h, 0.0f
		, -h, h, h, -h, h, -h, -h, -h, h
		, -h, -h, -h, -h, 0.0f, 0.0f
	};
	memcpy(machine->cube, cube, 3 * NUMBER_CUBE_PARTS * SIZE_FLOAT);*/

	v_set(&machine->cube[0], 0.0f, 0.0f, h);
	v_set(&machine->cube[1], 0.0f, 0.0f, -h);
	v_set(&machine->cube[2], 0.0f, h, 0.0f);
	v_set(&machine->cube[3], 0.0f, -h, 0.0f);
	v_set(&machine->cube[6], 0.0f, h, h);
	v_set(&machine->cube[7], 0.0f, h, -h);
	v_set(&machine->cube[8], 0.0f, -h, h);
	v_set(&machine->cube[9], 0.0f, -h, -h);

	v_set(&machine->cube[4], h, 0.0f, 0.0f);
	v_set(&machine->cube[10], h, 0.0f, h);
	v_set(&machine->cube[11], h, 0.0f, -h);
	v_set(&machine->cube[14], h, h, 0.0f);
	v_set(&machine->cube[15], h, -h, 0.0f);
	v_set(&machine->cube[18], h, h, h);
	v_set(&machine->cube[19], h, h, -h);
	v_set(&machine->cube[20], h, -h, h);
	v_set(&machine->cube[22], h, -h, -h);

	v_set(&machine->cube[5], -h, 0.0f, 0.0f);
	v_set(&machine->cube[12], -h, 0.0f, h);
	v_set(&machine->cube[13], -h, 0.0f, -h);
	v_set(&machine->cube[16], -h, h, 0.0f);
	v_set(&machine->cube[17], -h, -h, 0.0f);
	v_set(&machine->cube[21], -h, h, h);
	v_set(&machine->cube[23], -h, h, -h);
	v_set(&machine->cube[24], -h, -h, h);
	v_set(&machine->cube[25], -h, -h, -h);
	//v_set(&machine->cube[26], 0.0f, 0.0f, 0.0f);
}

void sph_create(sph_machine *machine, int number, sph_property *params, int max_neighbours, int grid_size)
{
	//v_set(&response, 0.0f, 0.0f, 0.0f);
	int i;

	machine->tag = (int*)malloc(sizeof(int) * NUMBER_PARTICLES);
	i = SIZE_INT_POOL;
	machine->int_memory_pool = (int*)malloc(SIZE_INT_POOL);
	sph_i_link = machine->int_memory_pool;
	if (sph_i_link == NULL)
	{
		exit(0);
	}	
	//NUMBER_PARTICLES = number;
	//MAX_NEIGHBOURS = max_neighbours;
	machine->vpc3f_memory_pool = (vector3f*)malloc(SIZE_VPC3F_POOL);
	sph_v_link = machine->vpc3f_memory_pool;
	if (sph_v_link == NULL)
	{
		free(sph_i_link);
		exit(0);
	}
	//machine->density = (float*)malloc(NUMBER_PARTICLES * sizeof(float));
	i = SIZE_FLOAT_POOL;
	machine->float_memory_pool = (float*)malloc(SIZE_FLOAT_POOL);
	sph_f_link = machine->float_memory_pool;
	if (sph_f_link == NULL)
	{
		free(sph_i_link);
		free(sph_v_link);
		exit(0);
	}
	//memset(sph_f_link, 0, SIZE_FLOAT_POOL);
	for (i = 0; i < (SIZE_FLOAT_POOL / SIZE_FLOAT); i++)
	{
		sph_f_link[i] = 0.0f;
	}

	/*H = params->smooth_length;
	REST_DENSITY = params->rest_density;
	STIFFNESS = params->stiffness;
	RESTITUTION = params->restitution;
	VISCOSITY = params->viscosity;
	TIMESTEP = params->timestep;
	P_MASS = params->mass;

	W_POLY6 = 315.0f / (64.0f * PI * powf(H, 9.0f));
	W_SPIKY = -45.0f / (PI * powf(H, 6.0f));
	W_DEFAULT = -945.0f /(32.0f * PI * powf(H, 9.0f));
	W_VISCOSITY = 45.0f / (PI * powf(H, 6.0f));
	H_H = H * H;*/
	
	/*machine->neighbours = (int**)malloc(SIZE_INT_LINK * number);
	for (i = 0; i < NUMBER_PARTICLES; i++)
	{
		machine->neighbours[i] = (int*)malloc(SIZE_INT * number);
		if (machine->neighbours[i] == NULL)
			exit(0);
	}*/
	//machine->distances = (float**)malloc(SIZE_FLOAT_LINK * number);
	machine->number_neighbours = (int*)malloc(SIZE_INT * number);
	memset(machine->number_neighbours, 0, SIZE_INT * number);
	
	set_cube(machine);
	sph_set_current_sph(machine);
	
	//v_set(GRAVITY, 0.0f, -9.8f, 0.0f);
	
	for (i = 0; i < NUMBER_PARTICLES; i++)
	{
		gs_mass[i] = P_MASS;
		v_set_zero(&gs_velocity[i]);
		v_set_zero(&gs_half_velocity[i]);
	}

	/* spatial partitioning */
#ifdef _HTNN
	ht_create(&machine->table, NUMBER_PARTICLES, H);
#endif
#ifdef _RGNN
	//i = (int)(2.0f * 0.2f / H) + 2;
	rg_create(&machine->grid, grid_size * grid_size * grid_size, H);
#endif

	iteration = 0;
	
#ifdef _DEBUG

	common_create_log_file();

#endif
}

void sph_destroy(sph_machine *machine)
{
	//int i;
	
	/*for (i = 0; i < NUMBER_PARTICLES; i++)
	{
		if (gs_number_neighbours[i] > 0)
		{
			free(gs_neighbours[i]);
			free(gs_distances[i]);
		}
	}*/
	//free(gs_neighbours);
	//free(gs_distances);
	free(gs_number_neighbours);
	free(sph_f_link);
	free(sph_v_link);
	free(sph_i_link);
	//free(machine->density);
	free(machine->tag);
#ifdef _HTNN 
	ht_destroy(&machine->table); 
#endif
#ifdef _RGNN 
	rg_destroy(&machine->grid);
#endif

#ifdef _DEBUG

	common_destroy_log_file();

#endif
}

void sph_set_gravity(vector3f *g)
{
	v_set(GRAVITY, g->x, g->y, g->z);
}

void sph_set_position(float interval, int xn, int yn, int zn)
{
	int x, y ,z;
	float xs, ys, zs;

	xs = (float)xn * interval * 0.5f;
	ys = (float)yn * interval * 0.5f;
	zs = (float)zn * interval * 0.5f;

	for (x = 0; x < xn; x++)
	{
		for (y = 0; y < yn; y++)
		{
			for (z = 0; z < zn; z++)
			{
				/*v_set(&g_position[x + (y * xn) + (z * xn * yn)]
					, -gs_sphere->radius / (float)xn + (interval * x)
					, -gs_sphere->radius / (float)yn + (interval * y)
					, -gs_sphere->radius / (float)zn + (interval * z));*/
				v_set(&g_position[x + (y * xn) + (z * xn * yn)]
					, -xs + (interval * (float)x)
					, -ys + (interval * (float)y)
					, -zs + (interval * (float)z));
			}
		}
	}
}

void sph_change_position(matrix44f rotation)
{
	int i;

	for (i = 0; i < NUMBER_PARTICLES; i++)
		m_multiplication_mv(&g_position[i], rotation);
}

void sph_set_collision_object(point3f *center, float radius)
{
	memcpy(&gs_sphere->center, center, SIZE_VPC3F);
	gs_sphere->radius = radius;
}

void sph_set_current_sph(sph_machine *machine)
{
	sph_i_link = machine->int_memory_pool;
	sph_f_link = machine->float_memory_pool;
	sph_v_link = machine->vpc3f_memory_pool;
	
	g_position = POSITION;
	gs_density = DENSITY;
	//gs_density = machine->density;
	gs_pressure = PRESSURE;
	gs_acceleration = ACCELERATION; 
	gs_f_pressure = F_PRESSURE;
	gs_f_viscosity = F_VISCOSITY;
	gs_mass = MASS;
	gs_velocity = VELOCITY;
	gs_half_velocity = HALF_VElOCITY;
	gs_cube = machine->cube;
	gs_sphere = &machine->sphere;
	//gs_distances = machine->distances;
	//gs_neighbours = machine->neighbours;
	gs_number_neighbours = machine->number_neighbours;
	gs_temp_distance = POOL_DISTANCE;
	gs_temp_index = POOL_INDEX;
	gs_sph = machine;

	//g_tag = machine->tag;
}

void sph_calculate_density_find_neighbours();
void sph_calculate_force_collision_detection();
void sph_update_velocity();

void sph_update()
{
	sph_calculate_density_find_neighbours();
	sph_calculate_force_collision_detection();
	sph_update_velocity();
	iteration++;
}

/* function that compute density and defines neighbours of each particle */

void sph_calculate_density_find_neighbours()
{
	//vector3f vector;
	int		i, j;
	//int		index;
	int		count;
	float	value;
	
	memset(gs_number_neighbours, 0, SIZE_INT * NUMBER_PARTICLES);
	memset(gs_density, 0, SIZE_FLOAT * NUMBER_PARTICLES);
	total = 0;
#ifdef _HTNN
	ht_update(&gs_sph->table);
	ht_find_neighbours(&gs_sph->table, gs_temp_index, gs_number_neighbours, gs_temp_distance);
#endif
#ifdef _RGNN
	rg_set_grid(&gs_sph->grid);
	rg_find_neighbours(&gs_sph->grid, gs_temp_index, gs_number_neighbours, gs_temp_distance);
#endif
	for (i = 0; i < NUMBER_PARTICLES; i++)
	{	
	/* find neighbours */
#ifdef _RGNN
		/* regular grid */
		for (j = 0; j < gs_number_neighbours[i]; j++)
		{
			value = H_H - gs_temp_distance[total];
			value = value * value * value;
			gs_density[i] += gs_mass[gs_temp_index[total]] * value;
			if (i != gs_temp_index[total])
				gs_density[gs_temp_index[total]] += gs_mass[i] * value;
			total++;
		}
#endif
		/* hash tables */
#ifdef _HTNN
		for (j = 0; j < gs_number_neighbours[i]; j++)
		{
			value = H_H - gs_temp_distance[total];
			value = value * value * value;
			gs_density[i] += gs_mass[gs_temp_index[total]] * value;
			if (i != gs_temp_index[total])
				gs_density[gs_temp_index[total]] += gs_mass[i] * value;
			total++;
		}
#endif
		/* brute force */
#ifdef _BFNN
		count = 0;
		for (j = 0; j < NUMBER_PARTICLES; j++)
		{
			v_subtract_d(&vector, &g_position[i], &g_position[j]);
			value = v_squared_magnitude(&vector);
			if (value < H_H)
			{
				gs_temp_index[total] = j;
				gs_temp_distance[total] = value;
				/* calculate density */
				value = H_H - value;
				value = value * value * value;
				gs_density[i] += gs_mass[j] * value;
				if (i != j)
					gs_density[j] += gs_mass[i] * value;
				count++;
				total++;
				if (count >= MAX_NEIGHBOURS)
				{
					break;
				}
			}
		}
		gs_number_neighbours[i] = count;
#endif
	}
	max_density = MIN_FLOAT;
	for (i = 0; i < NUMBER_PARTICLES; i++)
	{
		gs_density[i] *= W_POLY6;
		gs_pressure[i] = STIFFNESS * (gs_density[i] - REST_DENSITY);
		gs_density[i] = 1.0f / gs_density[i];
		if (gs_density[i] > max_density)
			max_density = gs_density[i];
	}
}

/* function that compute pressure, viscosity and collisions */

void sph_calculate_force_collision_detection()
{
	int i, j, id;
	vector3f vector, force;//, normal;
	float hr, length;//, hr2;

	total = 0;
	v_set_zero(&response);
	for (i = 0; i < NUMBER_PARTICLES; i++)
	{
		v_set_zero(&gs_acceleration[i]);
	}
	for (i = 0; i < NUMBER_PARTICLES; i++)
	{
		//v_set_zero(&normal);
		for (j = 0; j < gs_number_neighbours[i]; j++)
		{
			id = gs_temp_index[total];
			length = sqrtf(gs_temp_distance[total]);
			//length = v_magnitude(&vector);
			if (length < H && length > 0.0f)
			{
				hr = H - length; 
				v_subtract_d(&vector, &g_position[i], &g_position[id]);
				/*---------test---------*/
				/*hr2 = H_H - gs_temp_distance[total];
				v_scale_add_c(&normal, &vector, gs_mass[id] * gs_density[id] * W_DEFAULT * hr2 * hr2);*/
				/*---------test---------*/
				v_scale_d(&force, &vector, 
						-0.5f * (gs_pressure[i] + gs_pressure[id]) * W_SPIKY * hr / length);
				v_subtract_d(&vector, &gs_velocity[id], &gs_velocity[i]);
				v_scale_c(&vector, VISCOSITY * W_VISCOSITY);
				v_addition_c(&force, &vector);
				v_scale_c(&force, hr * gs_density[i] * gs_density[id]);
				/*---------test---------*/
				//hr = H_H - gs_temp_distance[total];
				//color += gs_mass[id] * gs_density[id] * W_POLY6 * hr * hr;
				/*---------test---------*/
				v_scale_add_c(&gs_acceleration[i], &force, gs_mass[id]);
				v_scale_add_c(&gs_acceleration[id], &force, -gs_mass[i]);
			}
			total++;
			//_ASSERT(total < MAX_NEIGHBOURS * NUMBER_PARTICLES);
		}
		/*value = v_magnitude(&normal);
		if (value > 70.0f)
		{
			g_tag[i] = 1;
		}*/
	}
	for(i = 0; i < NUMBER_PARTICLES; i++)
	{
		v_scale_add_d(&vector, &g_position[i], &gs_half_velocity[i], TIMESTEP);
		if (cd_point_sphere(&gs_half_velocity[i], &vector, &g_position[i], gs_sphere))
		{
			v_addition_c(&gs_acceleration[i], &response);
		}
	}
}

void sph_update_velocity()
{
	int i;
	vector3f nv;
	//point3f np;
	
	for (i = 0; i < NUMBER_PARTICLES; i++)
	{
		v_addition_c(&gs_acceleration[i], GRAVITY);
		v_scale_add_d(&nv, &gs_half_velocity[i], &gs_acceleration[i], TIMESTEP);
		v_scale_add_c(&g_position[i], &nv, TIMESTEP);
		//v_scale_add_d(&np, &g_position[i], &nv, TIMESTEP);
		//if (cd_point_sphere(&nv, &np, &g_position[i], gs_sphere) == 0)
		//{
		//	
		//	memcpy(&g_position[i], &np, SIZE_VPC3F);
		//}
		v_addition_d(&gs_velocity[i], &gs_half_velocity[i], &nv);
		v_scale_c(&gs_velocity[i], 0.5f);
		memcpy(&gs_half_velocity[i], &nv, SIZE_VPC3F);
		
#ifdef _DEBUG
		//fprintf(log_file, "Position %d : %f, %f, %f\n", i, g_position[i].x, g_position[i].y, g_position[i].z);
		//fprintf(log_file, "Velocity %d : %f, %f, %f\n", i, gs_velocity[i].x, gs_velocity[i].y, gs_velocity[i].z);
#endif
	}
}