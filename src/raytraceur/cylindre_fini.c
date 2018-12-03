/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   cylindre_fini.c                                    :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: cbesse <marvin@42.fr>                      +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/05/09 17:03:44 by cbesse            #+#    #+#             */
/*   Updated: 2018/05/30 09:57:04 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "rt.h"
#include "object.h"

void			fcyl_rec(t_ray2 *ray, double t, t_fcylindre *fcyl, \
		t_record *rec)
{
	t_vecteur	uv;
	t_vecteur	oc;

	rec->t = t;
	rec->p = v_add(ray->ori, v_mult(ray->dir, rec->t));
	oc = v_less(rec->p, fcyl->base);
	uv = v_mult(fcyl->dir, v_dot(fcyl->dir, oc));
	rec->normal = v_normalize(v_less(oc, uv));
	rec->texture.x = 0;
	rec->texture.y = 0;
	rec->type = 5;
}

int				fcyl_test(t_ray2 *ray, t_fcylindre *fcyl, double t, t_record *r)
{
	t_vecteur	uv;
	t_vecteur	oc;
	t_vecteur	normal;
	t_vecteur	p;
	double		ok;

	p = v_add(ray->ori, v_mult(ray->dir, t));
	oc = v_less(p, fcyl->base);
	uv = v_mult(fcyl->dir, v_dot(fcyl->dir, oc));
	normal = v_normalize(v_less(oc, uv));
	p = v_less(p, (v_less(oc, uv)));
	ok = v_norm(v_less(p, fcyl->base));
	if (ok <= fcyl->size / 2)
	{
		fcyl_rec(ray, t, fcyl, r);
		return (1);
	}
	return (0);
}

int				hit_fcylbord(t_fcylindre *fcyl, t_ray2 *ray, double *min_max, \
		t_record *rec)
{
	int	t;
	int	p;

	if (rec->f == 1 && (t = hit_plan(fcyl->plan1, ray, min_max, rec)))
	{
		set_min_max(min_max[0], rec->t, min_max);
		return (t);
	}
	if (rec->f == 2 && (p = hit_plan(fcyl->plan2, ray, min_max, rec)))
	{
		set_min_max(min_max[0], rec->t, min_max);
		return (p);
	}
	return (0);
}

static double	set(t_ray2 *ray, t_fcylindre *fcyl, t_vecteur *x, t_vec3f *abc)
{
	double		disc;

	*x = v_less(ray->ori, fcyl->base);
	abc->x = v_dot(ray->dir, ray->dir) - pow(v_dot(ray->dir, fcyl->dir), 2);
	abc->y = 2 * (v_dot(ray->dir, *x) - v_dot(ray->dir, fcyl->dir) * \
			v_dot(*x, fcyl->dir));
	abc->z = v_dot(*x, *x) - pow(v_dot(*x, fcyl->dir), 2) - fcyl->radius * \
			fcyl->radius;
	disc = abc->y * abc->y - 4 * abc->x * abc->z;
	return (disc);
}

int				hit_fcylindre(t_fcylindre *fcyl, t_ray2 *ray, double *min_max, \
		t_record *rec)
{
	t_vecteur	x;
	double		disc;
	double		r;
	t_vec3f		abc;

	if ((disc = set(ray, fcyl, &x, &abc)) > 0)
	{
		r = (-1 * abc.y - sqrt(disc)) / (2 * abc.x);
		if (r < min_max[1] && r > min_max[0] && fcyl_test(ray, fcyl, r, rec))
			return (1);
		rec->f = 1;
		if (hit_fcylbord(fcyl, ray, min_max, rec))
			return (1);
		rec->f = 2;
		if (hit_fcylbord(fcyl, ray, min_max, rec))
			return (1);
		r = (-1 * abc.y + sqrt(disc)) / (2 * abc.x);
		if (r < min_max[1] && r > min_max[0] && fcyl_test(ray, fcyl, r, rec))
			return (1);
	}
	return (0);
}
