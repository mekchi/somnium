
#include "isosurface.h"

void mc_create(iso_data *data, float number, float size)
{
	data->number = number * number * number;
	data->volume = (float*)malloc(data->number * SIZE_FLOAT);
	if (data->volume == NULL)
		exit(0);
	data->cell_size = size;
}

void mc_destroy(iso_data *data)
{
	free(data->volume);
}


void mc_metaballs(regular_grid *grid, iso_data *data)
{
	int i;
	int x, y, z, xi, yi, zi;
	int yzn;
	int zn;
	float l = 1.0f / data->cell_size;
	float d;
	point3f o, p;

	float xmax = MIN_FLOAT, ymax = MIN_FLOAT, zmax = MIN_FLOAT;
	float xmin = MAX_FLOAT, ymin = MAX_FLOAT, zmin = MAX_FLOAT;
	
	for (i = 0; i < NUMBER_PARTICLES; i++)
	{
		if (g_position[i].x > xmax){xmax = g_position[i].x;}
		if (g_position[i].y > ymax){ymax = g_position[i].y;}
		if (g_position[i].z > zmax){zmax = g_position[i].z;}
		if (g_position[i].x < xmin){xmin = g_position[i].x;}
		if (g_position[i].y < ymin){ymin = g_position[i].y;}
		if (g_position[i].z < zmin){zmin = g_position[i].z;}
	}
	
	xmin -= data->cell_size;
	ymin -= data->cell_size;
	zmin -= data->cell_size;
	xmax += data->cell_size;
	ymax += data->cell_size;
	zmax += data->cell_size;
	
	v_set(&data->origin, xmin, ymin, zmin);

	data->width = (int)((xmax - xmin) / data->cell_size);
	data->height = (int)((ymax - ymin) / data->cell_size);
	data->depth = (int)((zmax - zmin) / data->cell_size);
	yzn = (data->height + 1) * (data->depth + 1);
	zn = data->depth + 1;

	for (i = 0; i <data->number ; i++)
		data->volume[i] = 0.0f;
	for (i = 0; i < NUMBER_PARTICLES; i++)
	{
		x = (int)((g_position[i].x - data->origin.x) / data->cell_size);
		y = (int)((g_position[i].y - data->origin.y) / data->cell_size);
		z = (int)((g_position[i].z - data->origin.z) / data->cell_size);
		v_set(&o, data->origin.x + ((float)x * data->cell_size),
			data->origin.y + ((float)y * data->cell_size),
			data->origin.z + ((float)z * data->cell_size));

		for (xi = -1; xi <= 2; xi++)
			for (yi = -1; yi <= 2; yi++)
				for (zi = -1; zi <= 2; zi++)
				{
					p.x = o.x + (float)xi * data->cell_size;
					p.y = o.y + (float)yi * data->cell_size;
					p.z = o.z + (float)zi * data->cell_size;
					x = (int)((p.x - data->origin.x) * l);
					y = (int)((p.y - data->origin.y) * l);
					z = (int)((p.z - data->origin.z) * l);
					if (x < data->width && x >= 0 && y < data->height && y >= 0
						&& z < data->depth && z >= 0)
					{
						v_subtract_c(&p, &g_position[i]);
						d = v_squared_magnitude(&p);
						d = PORZ(H_H - d);
						d = W_POLY6 * d * d * d;
						data->volume[x * yzn + y * zn + z] += d;
					}
				}
	}
}