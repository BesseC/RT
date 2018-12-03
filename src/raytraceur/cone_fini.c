/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   cone_fini.c                                        :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: cbesse <marvin@42.fr>                      +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/05/10 19:07:50 by cbesse            #+#    #+#             */
/*   Updated: 2018/05/30 10:35:12 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "rt.h"
#include "object.h"

void			fcone_rec(t_ray2 *ray, double t, t_fcone *cone, t_record *rec)
{
	double		uv;
	t_vecteur	temp;
	t_vecteur	oc;

	rec->t = t;
	rec->p = v_add(ray->ori, v_mult(ray->dir, rec->t));
	oc = v_less(rec->p, cone->apex);
	if (v_dot(v_normalize(cone->dir), v_normalize(oc)) > 0)
		temp = v_set(cone->dir.x, cone->dir.y, cone->dir.z);
	else
		temp = v_set(-cone->dir.x, -cone->dir.y, -cone->dir.z);
	uv = v_norm(oc) / cos(cone->angle / 2);
	rec->normal = v_normalize(v_less(oc, v_mult(temp, uv)));
	if (v_dot(ray->dir, rec->normal) > 0)
		rec->inside = 1;
	rec->texture.x = 0;
	rec->type = 6;
	rec->texture.y = 0;
}

int				fcone_test(t_ray2 *ray, t_fcone *fcone, double t, t_record *rec)
{
	double		ok;
	t_vecteur	p;

	p = v_add(ray->ori, v_mult(ray->dir, t));
	ok = v_norm(v_less(v_add(ray->ori, v_mult(ray->dir, t)), fcone->apex)) * \
		cos(fcone->angle / 2);
	ok = v_dot(fcone->dir, v_normalize(v_less(p, fcone->apex))) < 0 ? -ok : ok;
	if (ok <= fcone->mid + fcone->size / 2 && ok >= fcone->mid - \
			fcone->size / 2)
	{
		fcone_rec(ray, t, fcone, rec);
		return (1);
	}
	return (0);
}

int				hit_fconebord(t_fcone *fcone, t_ray2 *ray, double *min_max, \
		t_record *rec)
{
	int	t;
	int	p;

	if (fcone->plan1->size != 0)
		if (rec->f == 1 && (t = hit_plan(fcone->plan1, ray, min_max, rec)))
		{
			set_min_max(min_max[0], rec->t, min_max);
			return (t);
		}
	if (fcone->plan2->size != 0)
		if (rec->f == 2 && (p = hit_plan(fcone->plan2, ray, min_max, rec)))
		{
			set_min_max(min_max[0], rec->t, min_max);
			return (p);
		}
	return (0);
}

static double	set(t_vecteur *x, t_fcone *fcone, t_ray2 *ray, t_vec3f *abc)
{
	double		k;

	k = tan(fcone->angle / 2);
	k = k * k;
	*x = v_less(ray->ori, fcone->apex);
	abc->x = v_dot(ray->dir, ray->dir) - (1 + k) * pow(v_dot(ray->dir, \
				fcone->dir), 2);
	abc->y = 2 * (v_dot(ray->dir, *x) - (1 + k) * v_dot(ray->dir, fcone->dir) \
			* v_dot(*x, fcone->dir));
	abc->z = v_dot(*x, *x) - (1 + k) * (pow(v_dot(*x, fcone->dir), 2));
	return (abc->y * abc->y - 4 * abc->x * abc->z);
}

int				hit_fcone(t_fcone *fcone, t_ray2 *ray, double *min_max, \
		t_record *rec)
{
	double		disc;
	double		r;
	t_vecteur	x;
	t_vec3f		abc;

	if ((disc = set(&x, fcone, ray, &abc)) > 0)
	{
		r = (-1 * abc.y - sqrt(disc)) / (2 * abc.x);
		if (r < min_max[1] && r > min_max[0] && fcone_test(ray, fcone, r, rec))
			return (1);
		rec->f = 1;
		if (hit_fconebord(fcone, ray, min_max, rec))
			return (1);
		rec->f = 2;
		if (hit_fconebord(fcone, ray, min_max, rec))
			return (1);
		r = (-1 * abc.y + sqrt(disc)) / (2 * abc.x);
		if (r < min_max[1] && r > min_max[0] && fcone_test(ray, fcone, r, rec))
			return (1);
	}
	return (0);
}
