
#ifndef _MEKCHI_SPATIAL_PARTITIONING_
#define _MEKCHI_SPATIAL_PARTITIONING_

#include "common.h"
//#include "physics.h"

#define PRIME_NUMBER_1 73856093
#define PRIME_NUMBER_2 19349663
#define PRIME_NUMBER_3 83492791

#define MEMORY_SIZE 25

struct t_hash_cell;
typedef struct t_hash_cell hash_cell;

struct t_hash_cell
{
	int			index;
	hash_cell	*link;
};

typedef struct
{
	hash_cell	**cells;
	hash_cell	**current_cells;
	int			number;
	float		length;

	
	//int			current_index;
	//int			memory_size;

	//int *last_indices;
	//int *current_indices;
	//int **indices;

	hash_cell	*pool;
	
} hash_table;

#define SIZE_HASH_CELL		sizeof(hash_cell)
#define SIZE_HASH_CELL_LINK	sizeof(hash_cell*)

void ht_create(hash_table *table, int number, float h);
void ht_destroy(hash_table *table);
void ht_update(hash_table *table);
void ht_find_neighbours(hash_table *table, int *neighbours, int *count, float *distances);


/* regular grid */

struct t_rg_cell;
typedef struct t_rg_cell rg_cell;

struct t_rg_cell
{
	int			index;
	rg_cell		*link;
};

#define SIZE_RG_CELL		sizeof(rg_cell)
#define SIZE_RG_CELL_LINK	sizeof(rg_cell*)

typedef struct
{
	int x;
	int y;
	int z;
} cell_index;

#define SIZE_CELL_INDEX		sizeof(cell_index)

typedef struct
{
	rg_cell		**cells;
	rg_cell		**current_cells;
	//int			*indices;
	cell_index	*indices;
	
	int			number;
	
	float		cell_size;
	
	point3f		origin;

	int			xn;
	int			yn;
	int			zn;

	float		width;
	float		height;
	float		depth;

	rg_cell		*pool;
	//int			pool_index;

} regular_grid;

void	rg_create(regular_grid *grid, int number_cells, float cell_size);
void	rg_destroy(regular_grid *grid);
int		rg_get_index(regular_grid *grid, int index);
int		rg_set_grid(regular_grid *grid);
int		rg_find_neighbours(regular_grid *grid, int *neighbours, int *count, float *distances);

#endif