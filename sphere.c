/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   sphere.c                                           :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: cbesse <marvin@42.fr>                      +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/04/26 16:00:27 by cbesse            #+#    #+#             */
/*   Updated: 2018/04/26 16:00:29 by cbesse           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "rt.h"

void	sphere_rec(t_ray *ray, double t, t_sphere *s, t_record *rec)
{
	rec->t = t;
	rec->p = v_add(ray->ori, v_mult(ray->dir, rec->t));
	rec->normal = v_div(v_less(rec->p, s->center), s->radius);
	if (v_dot(ray->dir, rec->normal) > 0)
	{
		//rec->normal.x = -rec->normal.x;
		//rec->normal.y = -rec->normal.y;
		//rec->normal.z = -rec->normal.z;
		rec->inside = 1;
	}
}

int	hit_sphere(t_sphere *sphere, t_ray *ray, double *min_max, t_record *rec)
{
	t_vecteur	oc;
	double		tab[5];

	oc = v_less(ray->ori, sphere->center);
	tab[0] = v_dot(ray->dir, ray->dir);
	tab[1] = 2.0 * v_dot(oc, ray->dir);
	tab[2] = v_dot(oc, oc) - sphere->radius * sphere->radius;
	tab[3] = tab[1] * tab[1] - 4 * tab[0] * tab[2];
	if (tab[3] > 0)
	{
		tab[4] = (-1 * tab[1] - sqrt(tab[3])) / (2 * tab[0]);
		if (tab[4] < min_max[1] && tab[4] > min_max[0])
		{
			sphere_rec(ray, tab[4], sphere, rec);
			return (1);
		}
		tab[4] = (-1 * tab[1] + sqrt(tab[3])) / (2 * tab[0]);
		if (tab[4] < min_max[1] && tab[4] > min_max[0])
		{
			sphere_rec(ray, tab[4], sphere, rec);
			return (1);
		}
	}
	return (0);
}
