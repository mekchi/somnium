
#ifndef _MEKCHI_MATH_
#define _MEKCHI_MATH_

//#ifdef __cplusplus
//extern "C"
//{
//#endif

#include <math.h>

#define PI				3.14159265358979323846f
#define ALMOST_NULL		0.0000000001f
#define RTOD(x)			((x) * 57.295f)
#define DTOR(x)			((x) * 0.017453f)
#define MAX_FLOAT		100000.0f 
#define MIN_FLOAT		-100000.0f
#define	ABS(x)			((x) < 0 ? (-(x)) : (x))
#define PORZ(x)			((x) < 0 ? (0) : (x))
#define SIZE_INT		sizeof(int)
#define SIZE_INT_LINK	sizeof(int*)
#define SIZE_FLOAT		sizeof(float)
#define SIZE_FLOAT_LINK sizeof(float*)

enum {E11, E21, E31, E41, E12, E22, E32, E42, 
		E13, E23, E33, E43, E14, E24, E34, E44};

typedef unsigned int uint;

typedef union
{
	struct
	{
		float x, y, z; 
	};
	struct
	{
		float r, g, b; 
	};
	float xyz[3];
} vector3f;

typedef vector3f point3f;
typedef vector3f color3f;

typedef struct
{
	float x, y, z, w;
} quat4f;

typedef float matrix44f[16];

#define	SIZE_VPC3F		sizeof(vector3f)
#define SIZE_MATRIX44	sizeof(float) * 16

/* vector */

void		v_set_zero(vector3f *vector);
void		v_set(vector3f *vector, float x, float y, float z);
float		v_magnitude(vector3f *vector);
float		v_squared_magnitude(vector3f *vector);
void		v_addition_c(vector3f *result, vector3f *vector);
void		v_addition_d(vector3f *result, vector3f *a, vector3f *b);
void		v_subtract_c(vector3f *result, vector3f *vector);
void		v_subtract_d(vector3f *result, vector3f *a, vector3f *b);
void		v_scale_c(vector3f *vector, float scale);
void		v_scale_d(vector3f *result, vector3f *vector, float scale);
void		v_scale_add_c(vector3f *result, vector3f *vector, float scale);
void		v_scale_add_d(vector3f *result, vector3f *a, vector3f *bs, float scale);
void		v_normalize_c(vector3f *vector);
void		v_normalize_d(vector3f *result, vector3f *vector);
float		v_dot_product(vector3f *a, vector3f *b);
void		v_cross_product(vector3f *result, vector3f *a, vector3f *b);
void		v_axis_to_quat(quat4f *result, vector3f *a, float phi);


/* quaternion */

void		q_set_zero(quat4f *q);
quat4f		q_get_normal(quat4f *q);
void		q_normalize(quat4f *q);
void		q_to_matrix(float *m, quat4f *q);
void		q_multiply_d(quat4f *result, quat4f *a, quat4f *b);

quat4f		qRotation(vector3f *a, vector3f *b);

/* matrix */

void	m_set_identity(matrix44f matrix);
void	m_set_trasnlation(matrix44f matrix, vector3f *vector);
void	m_invert_c(matrix44f matrix);
void	m_invert_d(matrix44f result, matrix44f matrix);
float	m_determinant(matrix44f matrix);
void	m_scale(matrix44f matrix, float scale);
void	m_multiplication_mv(vector3f *vector, matrix44f matrix);
void	m_transpose(matrix44f matrix);

//#ifdef __cplusplus
//}
//#endif

#endif