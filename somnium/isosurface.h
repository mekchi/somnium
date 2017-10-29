
#ifndef _MEKCHI_ISOSURFACE_
#define _MEKCHI_ISOSURFACE_

#include "common.h"
#include "spatial_partitioning.h"

typedef struct
{
	int		width;
	int		height;
	int		depth;
	float	cell_size;
	point3f origin;
	int		number;
	float	*volume;
} iso_data;

void mc_create(iso_data *data, float number, float size);
void mc_destroy(iso_data *data);
void mc_metaballs(regular_grid *grid, iso_data *data);


#endif