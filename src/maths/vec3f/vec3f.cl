typedef struct	s_vec3f {
	double x;
	double y;
	double z;
}		t_vec3f;

typedef struct	s_33mat{
	t_vec3f	up;
	t_vec3f	right;
	t_vec3f	forward;
	t_vec3f	transpose;
}		t_33mat;

__kernel void vec3f_add(__global t_vec3f *v1, __global t_vec3f *v2, __global t_vec3f *vo)
{
	vo->x = v1->x + v2->x;
	vo->y = v1->y + v2->y;
	vo->z = v1->z + v2->x;
}

__kernel void vec3f_sub(__global t_vec3f *v1, __global t_vec3f *v2, __global t_vec3f *vo)
{
	vo->x = v1->x - v2->x;
	vo->y = v1->y - v2->y;
	vo->z = v1->z - v2->x;
}

__kernel void vec3f_mul_unit(__global t_vec3f *v1, __global double *d, __global t_vec3f *vo)
{
	vo->x = v1->x * *d;
	vo->y = v1->y * *d;
	vo->z = v1->z * *d;
}

__kernel void vec3f_length2(__global t_vec3f *a, __global double *d)
{
	*d = a->x * a->x + a->y * a->y + a->z * a->z;
}

__kernel void vec3f_length3(__global t_vec3f *a, __global double *d)
{
	t_vec3f	length2(a, d);
	*d = sqrt(*d);
}

__kernel void vec3f_cpy(__global t_vec3f *a, __global t_vec3f *b)
{
	a->x = b->x;
	a->y = b->y;
	a->z = b->z;
}

__kernel void vec3f_dot(__global t_vec3f *a, __global t_vec3f *b, __global double *o)
{
	*o = a->x * b->x + a->y * b->y + a->z * b->z;
}

__kernel void vec3f_negate(__global t_vec3f *a, __global t_vec3f *b)
{
	b->x = -a->x;
	b->y = -a->y;
	b->z = -a->z;
}

__kernel void vec3f_normalize(__global t_vec3f *a, __global t_vec3f *b)
{
	double length;

	length = sqrt(a->x * a->x + a->y * a->y + a->z * a->z);
	b->x = a->x * 1 / length;
	b->y = a->y * 1 / length;
	b->z = a->z * 1 / length;
}

__kernel void vec3f_normalize2(__global t_vec3f *a, __global t_vec3f *b)
{
	unsigned int i = get_global_id(0);
	double length;

	vec3f_normalize(&a[i], &b[i]);
}
