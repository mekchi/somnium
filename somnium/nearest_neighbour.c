
#include "nearest_neighbour.h"

static int buffer_index = 0;

void nn_create(neighbourhood *list, int number)
{
	list->number = number;
	list->neighbours = (neighbour**)malloc(SIZE_NEIGHBOUR_LINK * number);
	list->buffer = (neighbour*)malloc(SIZE_NEIGHBOUR * number);
	buffer_index = 0;
}

void nn_destroy(neighbourhood *list)
{
	int i;

	/*for (i = 0; i < list->number; i++)
	{
		free(list->neighbours[i]);
	}*/
	nn_clear(list);
	free(list->neighbours);
	free(list->buffer);
}

void nn_clear(neighbourhood *list)
{
	int i;

	for (i = 0; i < list->number; i++)
	{
		free(list->neighbours[i]);
	}
}

void nn_find(sph_machine *machine)
{
	
	

}