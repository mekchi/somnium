
#include "spatial_partitioning.h"

//static hash_table	*current_table = NULL;
//static float		length = 0.0f;
//static int			last = 0;
static int			current_index = 0;

static int get_prime(int n)
{
	int n2 = n + 1;
	int found = 1;

	while(found)
	{
		if(n2%2 == 0)
		{
			n2++;
			continue;
		}
		else
		{	
			if(n2%3 == 0)
			{
				n2++;		
				continue;
			}
			else
			{
				if(n2%5 == 0)
				{
					n2++;
					continue;
				}
				else
				{
					found = 0;
				}
			}
		}
	}

	return n2;
}

void ht_create(hash_table *table, int number, float h)
{
	table->number = get_prime(2 * number);
	table->cells = (hash_cell**)malloc(SIZE_HASH_CELL_LINK * table->number);
	if (table->cells == NULL)
		exit(0);
	table->current_cells = (hash_cell**)malloc(SIZE_HASH_CELL_LINK * table->number);
	if (table->current_cells == NULL)
		exit(0);
	table->pool = (hash_cell*)malloc(SIZE_HASH_CELL * number);
	if (table->pool == NULL)
		exit(0);
	table->length = h;
}

void ht_destroy(hash_table *table)
{
	free(table->cells);
	free(table->current_cells);
	free(table->pool);
}

static void ht_clear(hash_table *table)
{
	int i;

	for (i = 0; i < table->number; i++)
	{
		table->cells[i] = NULL;
		table->current_cells[i] = NULL;
	}
}

static int ht_get_index(hash_table *table, point3f *p)
{
	int index = (((int)(p->x / table->length) * PRIME_NUMBER_1) 
			^ ((int)(p->y / table->length) * PRIME_NUMBER_2) 
			^ ((int)(p->z / table->length) * PRIME_NUMBER_3)) % table->number;

	return index < 0 ? index + table->number : index;
	//return index < 0 ? -index : index;
}

void ht_update(hash_table *table)
{
	int i, j;
	int mem = 0;

	ht_clear(table);
	for (i = 0; i < NUMBER_PARTICLES; i++)
	{
		j = ht_get_index(table, &g_position[i]);
		if (table->current_cells[j] == NULL)
		{
			table->current_cells[j] = &table->pool[mem];
			table->cells[j] = &table->pool[mem];
			mem++;
			table->current_cells[j]->index = i;
			table->current_cells[j]->link = NULL;
		}
		else
		{
			table->current_cells[j]->link = &table->pool[mem];
			mem++;
			table->current_cells[j] = table->current_cells[j]->link;
			table->current_cells[j]->index = i;
			table->current_cells[j]->link = NULL;
		}
	}
}

void ht_find_neighbours(hash_table *table, int *neighbours, int *count, float *distances)
{
	float d, h = table->length;
	vector3f v;
	hash_cell *c;
	int i, j, k = 0;
	int x, y, z;

	for (i = 0; i < NUMBER_PARTICLES; i++)
	{
		count[i] = 0;
		for (x = -1; x <= 1; x++)
			for (y = -1; y <= 1; y++)
				for (z = -1; z <= 1; z++)
				{
					v_set(&v, (float)x * h, (float)y * h, (float)z * h);
					v_addition_c(&v, &g_position[i]);
					j = ht_get_index(table, &v);
					c = table->cells[j];
					while(c != NULL)
					{
						v_subtract_d(&v, &g_position[i], &g_position[c->index]);
						d = v_squared_magnitude(&v);
						if (d < H_H)
						{
							distances[k] = d;
							neighbours[k] = c->index;
							count[i]++;
							k++;
						}
						c = c->link;
					}
				}
	}
}


/* regular grid */

void rg_create(regular_grid *grid, int number_cells, float cell_size)
{
	grid->pool = (rg_cell*)malloc(SIZE_RG_CELL * NUMBER_PARTICLES);
	if (grid->pool == NULL)
		exit(0);
	grid->cells = (rg_cell**)malloc(SIZE_RG_CELL_LINK * number_cells);
	if (grid->cells == NULL)
		exit(0);
	grid->current_cells = (rg_cell**)malloc(SIZE_RG_CELL_LINK * number_cells);
	if (grid->current_cells == NULL)
		exit(0);
	//grid->indices = (int*)malloc(SIZE_INT * NUMBER_PARTICLES);
	grid->indices = (cell_index*)malloc(SIZE_CELL_INDEX * NUMBER_PARTICLES);
	if (grid->indices == NULL)
		exit(0);
	
	grid->cell_size = cell_size;
	grid->number = number_cells;
	//grid->pool_index = 0;
	current_index = 0;
}

void rg_destroy(regular_grid *grid)
{
	free(grid->pool);
	free(grid->cells);
	free(grid->indices);
	free(grid->current_cells);
}

void rg_clear(regular_grid *grid)
{
	int i;

	for (i = 0; i < grid->number; i++)
	{
		grid->cells[i] = NULL;
		grid->current_cells[i] = NULL;
	}
	//grid->pool_index = 0;
	current_index = 0;
}

int rg_get_index(regular_grid *grid, int index)
{
	grid->indices[index].x = (int)((g_position[index].x - grid->origin.x) / grid->cell_size);
	grid->indices[index].y = (int)((g_position[index].y - grid->origin.y) / grid->cell_size);
	grid->indices[index].z = (int)((g_position[index].z - grid->origin.z) / grid->cell_size);

	return grid->indices[index].x * grid->yn * grid->zn 
		+ grid->indices[index].y * grid->zn + grid->indices[index].z;
}

int rg_set_grid(regular_grid *grid)
{
	int i, j, k = 0;
	float xmax = MIN_FLOAT, ymax = MIN_FLOAT, zmax = MIN_FLOAT;
	float xmin = MAX_FLOAT, ymin = MAX_FLOAT, zmin = MAX_FLOAT;
	rg_cell *c;
	
	rg_clear(grid);

	for (i = 0; i < NUMBER_PARTICLES; i++)
	{
		if (g_position[i].x > xmax){xmax = g_position[i].x;}
		if (g_position[i].y > ymax){ymax = g_position[i].y;}
		if (g_position[i].z > zmax){zmax = g_position[i].z;}
		if (g_position[i].x < xmin){xmin = g_position[i].x;}
		if (g_position[i].y < ymin){ymin = g_position[i].y;}
		if (g_position[i].z < zmin){zmin = g_position[i].z;}
	}
	
	xmin -= grid->cell_size;
	ymin -= grid->cell_size;
	zmin -= grid->cell_size;
	xmax += grid->cell_size;
	ymax += grid->cell_size;
	zmax += grid->cell_size;
	
	v_set(&grid->origin, xmin, ymin, zmin);

	grid->width = (xmax - xmin);
	grid->height = (ymax - ymin);
	grid->depth = (zmax - zmin);

	grid->xn = (int)(grid->width / grid->cell_size);
	grid->yn = (int)(grid->height / grid->cell_size);
	grid->zn = (int)(grid->depth / grid->cell_size);

	for (i = 0; i < NUMBER_PARTICLES; i++)
	{
		j = rg_get_index(grid, i);
		//grid->indices[i] = j;
		if (grid->current_cells[j] == NULL)
		{
			grid->current_cells[j] = &grid->pool[current_index];//grid->pool_index];
			grid->cells[j] = &grid->pool[current_index];//grid->pool_index];
			//grid->pool_index++;
			current_index++;
			grid->current_cells[j]->index = i;
			grid->current_cells[j]->link = NULL;
			k++;
		}
		else
		{
			grid->current_cells[j]->link = &grid->pool[current_index];//grid->pool_index];
			//grid->pool_index++;
			current_index++;
			grid->current_cells[j] = grid->current_cells[j]->link;
			grid->current_cells[j]->index = i;
			grid->current_cells[j]->link = NULL;
		}
	}
//#ifdef _DEBUG
//	for (i = 0; i < grid->number; i++)
//	{
//		if (grid->cells[i] != NULL)
//		{
//			c = grid->cells[i];
//			fprintf(log_file, "%d : ", i);
//			while(c != NULL)
//			{
//				fprintf(log_file, "%d, ", c->index);
//				c = c->link;
//			}
//			fprintf(log_file, "\n");
//		}
//	}
//#endif

	return k;
}

int rg_find_neighbours(regular_grid *grid, int *neighbours, int *count, float *distances)
{
	int i, ci, ni;
	int x, y, z;
	int k = 0;
	float d;
	rg_cell *c;
	vector3f v;
	
	for (i = 0; i < NUMBER_PARTICLES; i++)
	{
		ci = grid->indices[i].x * grid->yn * grid->zn 
			+ grid->indices[i].y * grid->zn + grid->indices[i].z;
		count[i] = 0;
		for (x = -1; x <= 1; x++)
			for (y = -1; y <= 1; y++)
				for (z = -1; z <= 1; z++)
				{
					ni = ci + x * grid->yn * grid->zn + y * grid->zn + z;
					if (ni < 0 || ni > grid->yn * grid->xn * grid->zn)
						continue;

					c = grid->cells[ni];
					while(c != NULL)
					{
						v_subtract_d(&v, &g_position[i], &g_position[c->index]);
						d = v_squared_magnitude(&v);
						if (d < H_H)
						{
							distances[k] = d;
							neighbours[k] = c->index;
							count[i]++;
							k++;
						}
						c = c->link;
					}
				}
	}

	return k;
}