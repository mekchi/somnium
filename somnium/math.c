#include "math.h"

/* vector */

void v_set(vector3f *vector, float x, float y, float z)
{
	vector->x = x;
	vector->y = y;
	vector->z = z;
}
void v_subtract_c(vector3f *result, vector3f *vector)
{
	result->x -= vector->x;
	result->y -= vector->y;
	result->z -= vector->z;
}

void v_subtract_d(vector3f *result, vector3f *a, vector3f *b)
{
	result->x = a->x - b->x;
	result->y = a->y - b->y;
	result->z = a->z - b->z;
}

void v_addition_c(vector3f *result, vector3f *vector)
{
	result->x += vector->x;
	result->y += vector->y;
	result->z += vector->z;
}

void v_addition_d(vector3f *result, vector3f *a, vector3f *b)
{
	result->x = a->x + b->x;
	result->y = a->y + b->y;
	result->z = a->z + b->z;
}

void v_set_zero(vector3f *vector)
{
	vector->x = 0.0f;
	vector->y = 0.0f;
	vector->z = 0.0f;
	//memset(vector, 0.0f, SIZE_FLOAT * 3); 
}

void v_scale_c(vector3f *vector, float scale)
{
	vector->x *= scale;
	vector->y *= scale;
	vector->z *= scale;
}

void v_scale_d(vector3f *result, vector3f *vector, float scale)
{
	result->x = vector->x * scale;
	result->y = vector->y * scale;
	result->z = vector->z * scale;
}

void v_scale_add_c(vector3f *result, vector3f *vector, float scale)
{
	result->x += vector->x * scale;
	result->y += vector->y * scale;
	result->z += vector->z * scale;
}

void v_scale_add_d(vector3f *result, vector3f *a, vector3f *bs, float scale)
{
	result->x = a->x + (bs->x * scale);
	result->y = a->y + (bs->y * scale);
	result->z = a->z + (bs->z * scale);
}

float v_magnitude(vector3f *vector)
{
	return sqrtf((vector->x * vector->x) + (vector->y * vector->y) + (vector->z * vector->z));
}

float v_squared_magnitude(vector3f *vector)
{
	return (vector->x * vector->x) + (vector->y * vector->y) + (vector->z * vector->z);
}

void v_normalize_c(vector3f *vector)
{
	float l = v_magnitude(vector);

	vector->x /= l;
	vector->y /= l;
	vector->z /= l;
}

void v_normalize_d(vector3f *result, vector3f *vector)
{
	float l = v_magnitude(vector);
	
	result->x /= l;
	result->y /= l;
	result->z /= l;
}

float v_dot_product(vector3f *a, vector3f *b)
{
	return ((a->x * b->x) + (a->y * b->y) + (a->z * b->z));
}

void v_cross_product(vector3f *result, vector3f *a, vector3f *b)
{
	result->x = (a->y * b->z) - (b->y * a->z);
	result->y = (a->z * b->x) - (b->z * a->x);
	result->z = (a->x * b->y) - (b->x * a->y);
}

void v_axis_to_quat(quat4f *result, vector3f *a, float phi)
{
	float alpha = sinf(phi);

	result->x = a->x * alpha;
	result->y = a->y * alpha;
	result->z = a->z * alpha;
	result->w = cosf(phi);
}

/* quaternion math */

void q_set_zero(quat4f *q)
{
	q->x = 0.0f;
	q->y = 0.0f;
	q->z = 0.0f;
	q->w = 1.0f;
}

quat4f q_get_normal(quat4f *q)
{
	quat4f norm;
	float m = sqrtf((q->x * q->x) + (q->y * q->y)
		+ (q->z * q->z) + (q->w * q->w));

	if (m > 0.0f)
	{
		norm.x = q->x / m;
		norm.y = q->y / m;
		norm.z = q->z / m;
		norm.w = q->w / m;
	}

	return norm;
}

void q_normalize(quat4f *q)
{
	float m = sqrtf((q->x * q->x) + (q->y * q->y)
		+ (q->z * q->z) + (q->w * q->w));

	if (m > 0.0f)
	{
		q->x /= m;
		q->y /= m;
		q->z /= m;
		q->w /= m;
	}
}

quat4f qRotation(vector3f *a, vector3f *b)
{
	// version 1
	//float val = vDotProduct(a, b);
	//float3 cv;

	//vCrossProduct(a, b, cv);
	//val = sqrtf(vSquaredMagnitude(a) * vSquaredMagnitude(b)) + val;
	//if (val < ALMOST_NULL)
	//{
	//	out[X] = -a[Z];
	//	out[Y] = a[Y];
	//	out[Z] = a[X];
	//	out[W] = 0.0f;
	//	//return;
	//}
	//else
	//{
	//	out[X] = cv[X];
	//	out[Y] = cv[Y];
	//	out[Z] = cv[Z];
	//	out[W] = val;
	//}
	//qNormalize(out);

	// version 2
	float dp = v_dot_product(a, b);
	vector3f cp;
	quat4f q;

	v_cross_product(&cp, a, b);
	
	q.x = cp.x;
	q.y = cp.y;
	q.z = cp.z;
	q.w = dp + sqrtf(v_squared_magnitude(a) * v_squared_magnitude(b));
	q_normalize(&q);

	return q;

	// version 3
	//float s = sqrtf(0.5f * (1.0f - vDotProduct(a, b)));
	//float3 cp;

	//vCrossProduct(a, b, cp);

	//out[X] = cp[X] * s;
	//out[Y] = cp[Y] * s;
	//out[Z] = cp[Z] * s;
	//out[W] = sqrtf(0.5f * (1.0f + vDotProduct(a, b)));
}

void q_multiply_d(quat4f *result, quat4f *a, quat4f *b)
{
	result->w = a->w * b->w - a->x * b->x - a->y * b->y - a->z * b->z;
	result->x = a->x * b->w + a->w * b->x + a->y * b->z - a->z * b->y;
	result->y = a->w * b->y - a->x * b->z + a->y * b->w + a->z * b->x;
	result->z = a->w * b->z + a->x * b->y - a->y * b->x + a->z * b->w;
}

void q_to_matrix(float *m, quat4f *q)
{
	m[E11] = 1.0f - 2.0f * (q->z * q->z + q->y * q->y);
	m[E22] = 1.0f - 2.0f * (q->z * q->z + q->x * q->x);
	m[E33] = 1.0f - 2.0f * (q->y * q->y + q->x * q->x);

	m[E12] = 2.0f * (q->x * q->y - q->z * q->w);
	m[E13] = 2.0f * (q->x * q->z + q->y * q->w);

	m[E21] = 2.0f * (q->x * q->y + q->w * q->z);
	m[E23] = 2.0f * (q->z * q->y - q->x * q->w);

	m[E31] = 2.0f * (q->x * q->z - q->w * q->y);
	m[E32] = 2.0f * (q->y * q->z + q->w * q->x);

	m[E14] = 0.0f;
	m[E24] = 0.0f;
	m[E34] = 0.0f;
	m[E41] = 0.0f;
	m[E42] = 0.0f;
	m[E43] = 0.0f;
	m[E44] = 1.0f;
}

/* matrix */

void m_set_identity(matrix44f matrix)
{
	matrix[E11] = 1.0f;
	matrix[E22] = 1.0f;
	matrix[E33] = 1.0f;
	matrix[E44] = 1.0f;
	matrix[E12] = 0.0f;
	matrix[E13] = 0.0f;
	matrix[E14] = 0.0f;
	matrix[E21] = 0.0f;
	matrix[E23] = 0.0f;
	matrix[E24] = 0.0f;
	matrix[E31] = 0.0f;
	matrix[E32] = 0.0f;
	matrix[E34] = 0.0f;
	matrix[E41] = 0.0f;
	matrix[E42] = 0.0f;
	matrix[E43] = 0.0f;
}

void m_set_trasnlation(matrix44f matrix, vector3f *vector)
{
	matrix[E11] = 1.0f;
	matrix[E22] = 1.0f;
	matrix[E33] = 1.0f;
	matrix[E44] = 1.0f;
	matrix[E12] = 0.0f;
	matrix[E13] = 0.0f;
	matrix[E14] = 0.0f;
	matrix[E21] = 0.0f;
	matrix[E23] = 0.0f;
	matrix[E24] = 0.0f;
	matrix[E31] = 0.0f;
	matrix[E32] = 0.0f;
	matrix[E34] = 0.0f;

	matrix[E41] = vector->x;
	matrix[E42] = vector->y;
	matrix[E43] = vector->z;
}

void m_invert_c(matrix44f matrix)
{
	matrix44f temp;
	
	temp[E11] = (matrix[E23] * matrix[E34] * matrix[E42]) - (matrix[E24] * matrix[E33] * matrix[E42]) + (matrix[E24] * matrix[E32] * matrix[E43]) - (matrix[E22] * matrix[E34] * matrix[E43]) - (matrix[E23] * matrix[E32] * matrix[E44]) + (matrix[E22] * matrix[E33] * matrix[E44]);
	temp[E12] = (matrix[E14] * matrix[E33] * matrix[E42]) - (matrix[E13] * matrix[E34] * matrix[E42]) - (matrix[E14] * matrix[E32] * matrix[E43]) + (matrix[E12] * matrix[E34] * matrix[E43]) + (matrix[E13] * matrix[E32] * matrix[E44]) - (matrix[E12] * matrix[E33] * matrix[E44]);
	temp[E13] = (matrix[E13] * matrix[E24] * matrix[E42]) - (matrix[E14] * matrix[E23] * matrix[E42]) + (matrix[E14] * matrix[E22] * matrix[E43]) - (matrix[E12] * matrix[E24] * matrix[E43]) - (matrix[E13] * matrix[E22] * matrix[E44]) + (matrix[E12] * matrix[E23] * matrix[E44]);
	temp[E14] = (matrix[E14] * matrix[E23] * matrix[E32]) - (matrix[E13] * matrix[E24] * matrix[E32]) - (matrix[E14] * matrix[E22] * matrix[E33]) + (matrix[E12] * matrix[E24] * matrix[E33]) + (matrix[E13] * matrix[E22] * matrix[E34]) - (matrix[E12] * matrix[E23] * matrix[E34]);
	temp[E21] = (matrix[E24] * matrix[E33] * matrix[E41]) - (matrix[E23] * matrix[E34] * matrix[E41]) - (matrix[E24] * matrix[E31] * matrix[E43]) + (matrix[E21] * matrix[E34] * matrix[E43]) + (matrix[E23] * matrix[E31] * matrix[E44]) - (matrix[E21] * matrix[E33] * matrix[E44]);
	temp[E22] = (matrix[E13] * matrix[E34] * matrix[E41]) - (matrix[E14] * matrix[E33] * matrix[E41]) + (matrix[E14] * matrix[E31] * matrix[E43]) - (matrix[E11] * matrix[E34] * matrix[E43]) - (matrix[E13] * matrix[E31] * matrix[E44]) + (matrix[E11] * matrix[E33] * matrix[E44]);
	temp[E23] = (matrix[E14] * matrix[E23] * matrix[E41]) - (matrix[E13] * matrix[E24] * matrix[E41]) - (matrix[E14] * matrix[E21] * matrix[E43]) + (matrix[E11] * matrix[E24] * matrix[E43]) + (matrix[E13] * matrix[E21] * matrix[E44]) - (matrix[E11] * matrix[E23] * matrix[E44]);
	temp[E24] = (matrix[E13] * matrix[E24] * matrix[E31]) - (matrix[E14] * matrix[E23] * matrix[E31]) + (matrix[E14] * matrix[E21] * matrix[E33]) - (matrix[E11] * matrix[E24] * matrix[E33]) - (matrix[E13] * matrix[E21] * matrix[E34]) + (matrix[E11] * matrix[E23] * matrix[E34]);
	temp[E31] = (matrix[E22] * matrix[E34] * matrix[E41]) - (matrix[E24] * matrix[E32] * matrix[E41]) + (matrix[E24] * matrix[E31] * matrix[E42]) - (matrix[E21] * matrix[E34] * matrix[E42]) - (matrix[E22] * matrix[E31] * matrix[E44]) + (matrix[E21] * matrix[E32] * matrix[E44]);
	temp[E32] = (matrix[E14] * matrix[E32] * matrix[E41]) - (matrix[E12] * matrix[E34] * matrix[E41]) - (matrix[E14] * matrix[E31] * matrix[E42]) + (matrix[E11] * matrix[E34] * matrix[E42]) + (matrix[E12] * matrix[E31] * matrix[E44]) - (matrix[E11] * matrix[E32] * matrix[E44]);
	temp[E33] = (matrix[E12] * matrix[E24] * matrix[E41]) - (matrix[E14] * matrix[E22] * matrix[E41]) + (matrix[E14] * matrix[E21] * matrix[E42]) - (matrix[E11] * matrix[E24] * matrix[E42]) - (matrix[E12] * matrix[E21] * matrix[E44]) + (matrix[E11] * matrix[E22] * matrix[E44]);
	temp[E34] = (matrix[E14] * matrix[E22] * matrix[E31]) - (matrix[E12] * matrix[E24] * matrix[E31]) - (matrix[E14] * matrix[E21] * matrix[E32]) + (matrix[E11] * matrix[E24] * matrix[E32]) + (matrix[E12] * matrix[E21] * matrix[E34]) - (matrix[E11] * matrix[E22] * matrix[E34]);
	temp[E41] = (matrix[E23] * matrix[E32] * matrix[E41]) - (matrix[E22] * matrix[E33] * matrix[E41]) - (matrix[E23] * matrix[E31] * matrix[E42]) + (matrix[E21] * matrix[E33] * matrix[E42]) + (matrix[E22] * matrix[E31] * matrix[E43]) - (matrix[E21] * matrix[E32] * matrix[E43]);
	temp[E42] = (matrix[E12] * matrix[E33] * matrix[E41]) - (matrix[E13] * matrix[E32] * matrix[E41]) + (matrix[E13] * matrix[E31] * matrix[E42]) - (matrix[E11] * matrix[E33] * matrix[E42]) - (matrix[E12] * matrix[E31] * matrix[E43]) + (matrix[E11] * matrix[E32] * matrix[E43]);
	temp[E43] = (matrix[E13] * matrix[E22] * matrix[E41]) - (matrix[E12] * matrix[E23] * matrix[E41]) - (matrix[E13] * matrix[E21] * matrix[E42]) + (matrix[E11] * matrix[E23] * matrix[E42]) + (matrix[E12] * matrix[E21] * matrix[E43]) - (matrix[E11] * matrix[E22] * matrix[E43]);
	temp[E44] = (matrix[E12] * matrix[E23] * matrix[E31]) - (matrix[E13] * matrix[E22] * matrix[E31]) + (matrix[E13] * matrix[E21] * matrix[E32]) - (matrix[E11] * matrix[E23] * matrix[E32]) - (matrix[E12] * matrix[E21] * matrix[E33]) + (matrix[E11] * matrix[E22] * matrix[E33]);
	m_scale(temp, (1.0f / m_determinant(matrix)));
	memcpy(matrix, temp, SIZE_MATRIX44);
}

void m_invert_d(matrix44f result, matrix44f matrix)
{	
	result[E11] = (matrix[E23] * matrix[E34] * matrix[E42]) - (matrix[E24] * matrix[E33] * matrix[E42]) + (matrix[E24] * matrix[E32] * matrix[E43]) - (matrix[E22] * matrix[E34] * matrix[E43]) - (matrix[E23] * matrix[E32] * matrix[E44]) + (matrix[E22] * matrix[E33] * matrix[E44]);
	result[E12] = (matrix[E14] * matrix[E33] * matrix[E42]) - (matrix[E13] * matrix[E34] * matrix[E42]) - (matrix[E14] * matrix[E32] * matrix[E43]) + (matrix[E12] * matrix[E34] * matrix[E43]) + (matrix[E13] * matrix[E32] * matrix[E44]) - (matrix[E12] * matrix[E33] * matrix[E44]);
	result[E13] = (matrix[E13] * matrix[E24] * matrix[E42]) - (matrix[E14] * matrix[E23] * matrix[E42]) + (matrix[E14] * matrix[E22] * matrix[E43]) - (matrix[E12] * matrix[E24] * matrix[E43]) - (matrix[E13] * matrix[E22] * matrix[E44]) + (matrix[E12] * matrix[E23] * matrix[E44]);
	result[E14] = (matrix[E14] * matrix[E23] * matrix[E32]) - (matrix[E13] * matrix[E24] * matrix[E32]) - (matrix[E14] * matrix[E22] * matrix[E33]) + (matrix[E12] * matrix[E24] * matrix[E33]) + (matrix[E13] * matrix[E22] * matrix[E34]) - (matrix[E12] * matrix[E23] * matrix[E34]);
	result[E21] = (matrix[E24] * matrix[E33] * matrix[E41]) - (matrix[E23] * matrix[E34] * matrix[E41]) - (matrix[E24] * matrix[E31] * matrix[E43]) + (matrix[E21] * matrix[E34] * matrix[E43]) + (matrix[E23] * matrix[E31] * matrix[E44]) - (matrix[E21] * matrix[E33] * matrix[E44]);
	result[E22] = (matrix[E13] * matrix[E34] * matrix[E41]) - (matrix[E14] * matrix[E33] * matrix[E41]) + (matrix[E14] * matrix[E31] * matrix[E43]) - (matrix[E11] * matrix[E34] * matrix[E43]) - (matrix[E13] * matrix[E31] * matrix[E44]) + (matrix[E11] * matrix[E33] * matrix[E44]);
	result[E23] = (matrix[E14] * matrix[E23] * matrix[E41]) - (matrix[E13] * matrix[E24] * matrix[E41]) - (matrix[E14] * matrix[E21] * matrix[E43]) + (matrix[E11] * matrix[E24] * matrix[E43]) + (matrix[E13] * matrix[E21] * matrix[E44]) - (matrix[E11] * matrix[E23] * matrix[E44]);
	result[E24] = (matrix[E13] * matrix[E24] * matrix[E31]) - (matrix[E14] * matrix[E23] * matrix[E31]) + (matrix[E14] * matrix[E21] * matrix[E33]) - (matrix[E11] * matrix[E24] * matrix[E33]) - (matrix[E13] * matrix[E21] * matrix[E34]) + (matrix[E11] * matrix[E23] * matrix[E34]);
	result[E31] = (matrix[E22] * matrix[E34] * matrix[E41]) - (matrix[E24] * matrix[E32] * matrix[E41]) + (matrix[E24] * matrix[E31] * matrix[E42]) - (matrix[E21] * matrix[E34] * matrix[E42]) - (matrix[E22] * matrix[E31] * matrix[E44]) + (matrix[E21] * matrix[E32] * matrix[E44]);
	result[E32] = (matrix[E14] * matrix[E32] * matrix[E41]) - (matrix[E12] * matrix[E34] * matrix[E41]) - (matrix[E14] * matrix[E31] * matrix[E42]) + (matrix[E11] * matrix[E34] * matrix[E42]) + (matrix[E12] * matrix[E31] * matrix[E44]) - (matrix[E11] * matrix[E32] * matrix[E44]);
	result[E33] = (matrix[E12] * matrix[E24] * matrix[E41]) - (matrix[E14] * matrix[E22] * matrix[E41]) + (matrix[E14] * matrix[E21] * matrix[E42]) - (matrix[E11] * matrix[E24] * matrix[E42]) - (matrix[E12] * matrix[E21] * matrix[E44]) + (matrix[E11] * matrix[E22] * matrix[E44]);
	result[E34] = (matrix[E14] * matrix[E22] * matrix[E31]) - (matrix[E12] * matrix[E24] * matrix[E31]) - (matrix[E14] * matrix[E21] * matrix[E32]) + (matrix[E11] * matrix[E24] * matrix[E32]) + (matrix[E12] * matrix[E21] * matrix[E34]) - (matrix[E11] * matrix[E22] * matrix[E34]);
	result[E41] = (matrix[E23] * matrix[E32] * matrix[E41]) - (matrix[E22] * matrix[E33] * matrix[E41]) - (matrix[E23] * matrix[E31] * matrix[E42]) + (matrix[E21] * matrix[E33] * matrix[E42]) + (matrix[E22] * matrix[E31] * matrix[E43]) - (matrix[E21] * matrix[E32] * matrix[E43]);
	result[E42] = (matrix[E12] * matrix[E33] * matrix[E41]) - (matrix[E13] * matrix[E32] * matrix[E41]) + (matrix[E13] * matrix[E31] * matrix[E42]) - (matrix[E11] * matrix[E33] * matrix[E42]) - (matrix[E12] * matrix[E31] * matrix[E43]) + (matrix[E11] * matrix[E32] * matrix[E43]);
	result[E43] = (matrix[E13] * matrix[E22] * matrix[E41]) - (matrix[E12] * matrix[E23] * matrix[E41]) - (matrix[E13] * matrix[E21] * matrix[E42]) + (matrix[E11] * matrix[E23] * matrix[E42]) + (matrix[E12] * matrix[E21] * matrix[E43]) - (matrix[E11] * matrix[E22] * matrix[E43]);
	result[E44] = (matrix[E12] * matrix[E23] * matrix[E31]) - (matrix[E13] * matrix[E22] * matrix[E31]) + (matrix[E13] * matrix[E21] * matrix[E32]) - (matrix[E11] * matrix[E23] * matrix[E32]) - (matrix[E12] * matrix[E21] * matrix[E33]) + (matrix[E11] * matrix[E22] * matrix[E33]);
	m_scale(result, (1.0f / m_determinant(matrix)));
}

float m_determinant(matrix44f matrix)
{
	return ((matrix[E14] * matrix[E23] * matrix[E32] * matrix[E41]) - (matrix[E13] * matrix[E24] * matrix[E32] * matrix[E41]) - (matrix[E14] * matrix[E22] * matrix[E33] * matrix[E41]) + (matrix[E12] * matrix[E24]
		* matrix[E33] * matrix[E41]) + (matrix[E13] * matrix[E22] * matrix[E34] * matrix[E41]) - (matrix[E12] * matrix[E23] * matrix[E34] * matrix[E41]) - (matrix[E14] * matrix[E23] * matrix[E31] * matrix[E42])
		+ (matrix[E13] * matrix[E24] * matrix[E31] * matrix[E42]) + (matrix[E14] * matrix[E21] * matrix[E33] * matrix[E42]) - (matrix[E11] * matrix[E24] * matrix[E33] * matrix[E42]) - (matrix[E13] * matrix[E21]
		* matrix[E34] * matrix[E42]) + (matrix[E11] * matrix[E23] * matrix[E34] * matrix[E42]) + (matrix[E14] * matrix[E22] * matrix[E31] * matrix[E43]) - (matrix[E12] * matrix[E24] * matrix[E31] * matrix[E43])
		- (matrix[E14] * matrix[E21] * matrix[E32] * matrix[E43]) + (matrix[E11] * matrix[E24] * matrix[E32] * matrix[E43]) + (matrix[E12] * matrix[E21] * matrix[E34] * matrix[E43]) - (matrix[E11] * matrix[E22]
		* matrix[E34] * matrix[E43]) - (matrix[E13] * matrix[E22] * matrix[E31] * matrix[E44]) + (matrix[E12] * matrix[E23] * matrix[E31] * matrix[E44]) + (matrix[E13] * matrix[E21] * matrix[E32] * matrix[E44])
		- (matrix[E11] * matrix[E23] * matrix[E32] * matrix[E44]) - (matrix[E12] * matrix[E21] * matrix[E33] * matrix[E44]) + (matrix[E11] * matrix[E22] * matrix[E33] * matrix[E44]));
}

void m_scale(matrix44f matrix, float scale)
{
	int i;

	for(i = 0; i < 16; i++)
	{
		matrix[i] *= scale;
	}
}

void m_multiplication_mv(vector3f *vector, matrix44f matrix)
{
	vector3f	temp;
	float		r = matrix[E41] * vector->x + matrix[E42] * vector->y + matrix[E43] * vector->z + matrix[E44];

	temp.x = (matrix[E11] * vector->x + matrix[E12] * vector->y + matrix[E13] * vector->z + matrix[E14]) / r;
	temp.y = (matrix[E21] * vector->x + matrix[E22] * vector->y + matrix[E23] * vector->z + matrix[E24]) / r;
	temp.z = (matrix[E31] * vector->x + matrix[E32] * vector->y + matrix[E33] * vector->z + matrix[E34]) / r;
	memcpy(vector, &temp, SIZE_VPC3F);
}

void m_transpose(matrix44f matrix)
{
	int i, j;
	matrix44f out;

	memcpy(out, matrix, SIZE_MATRIX44);
	for(i = 0; i < 4; i++)
		for(j = 0; j < 4; j++)
		{
			matrix[(i * 4) + j] = out[ i + (j * 4)];
		}
}