
#ifndef _MEKCHI_NEAREST_NEIGHBOUR_
#define _MEKCHI_NEAREST_NEIGHBOUR_

#include "physics.h"
#include "spatial_partitioning.h"

typedef struct
{
	int index;
	float distance;

} neighbour;

typedef struct 
{
	neighbour	**neighbours;
	int			*sizes;
	int			number;

	neighbour	*buffer;
} neighbourhood;

const int SIZE_NEIGHBOUR = sizeof(neighbour);
const int SIZE_NEIGHBOUR_LINK = sizeof(neighbour*);

void nn_create(neighbourhood *list, int number);
void nn_destroy(neighbourhood *list);
void nn_clear(neighbourhood *list);
void nn_find(neighbourhood *list);

#endif
