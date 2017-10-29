
#include "collision_detection.h"

//void cd_point_obb(point3f *point, obb *object))
//{
//	
//}


int	cd_point_sphere(vector3f *velocity, point3f *new_position, point3f *position, cd_sphere *sphere)
{
	vector3f n;
	float d;//, mv;
	
	v_subtract_d(&n, &sphere->center, new_position);
	d = v_magnitude(&n) - sphere->radius;
	if (d < 0.0f)
	{
		return 0;
	}
	else
	{
		/* acceleration based collision (using spring force) */
		v_normalize_c(&n);
		v_scale_d(&response, &n, (30000.0f * d - 128.0f * v_dot_product(&n, velocity)));
		
		
		/* impulse projection method */
		/*v_normalize_c(&n);
		v_scale_add_c(position, &n, d);
		mv = v_magnitude(velocity);
		v_scale_c(&n, (1.0f + (RESTITUTION * d) / (TIMESTEP * mv)) * v_dot_product(&n, velocity));
		v_subtract_c(velocity, &n);*/

		return 1;
	}


}