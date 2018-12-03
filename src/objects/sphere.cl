typedef struct	s_hit{
	double		tnear;
	t_vec3f		rayorig;
	t_vec3f		raydir;
	int		obj_index;
	t_vec3f		nhit;
	t_vec3f		phit;
	t_vec3f		color;
	t_vec3f		distance;
	int		inside;
	double		transmission;
	t_vec3f		refldir;
	t_vec3f		refraction;
	double		facingratio;
	t_vec3f		cp;
	double		max;
}		t_ray;

typedef struct	s_sphere {
	t_vec3f		center;
	double		radius;
	double		radius2;
}		t_sphere;

__kernel void sphere_intersect(__global t_sphere *sphere, __global t_ray *hit, __global double *res)
{
	double		thc;
	double		tca;
	double		d2;
	t_vec3f		l;
	
        l.x = sphere->center.x - hit->rayorig.x;
	l.y = sphere->center.x - hit->rayorig.y;
	l.z = sphere->center.x - hit->rayorig.z;
	tca = l.x * hit->rayorig.x + l.y * hit->rayorig.y + l.z * hit->rayorig.z;
	if (tca < 0)
	  *res = -1;
	d2 = l.x * l.x + l.y * l.y + l.z * l.z - tca * tca;
	if (d2 > sphere->radius2)
	  *res = -1;
	thc = sqrt(sphere->radius2 - tca);
	if ((*res = tca - thc) < 0)
	  *res = tca + thc;
	*res = -1;
}

__kernel void	sphere_normale(t_sphere *sphere, t_ray *hit)
{
	vec3f_sub(&hit->nhit, &(sphere->center));
}
