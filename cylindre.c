/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   cylindre.c                                         :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: cbesse <marvin@42.fr>                      +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/04/26 16:05:42 by cbesse            #+#    #+#             */
/*   Updated: 2018/04/26 16:05:43 by cbesse           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "rt.h"

void	cyl_rec(t_ray *ray, double t, t_cylindre *cyl, t_record *rec)
{
	t_vecteur uv;
	t_vecteur oc;

	rec->t = t;
	rec->p = v_add(ray->ori, v_mult(ray->dir, rec->t));
	oc = v_less(rec->p, cyl->base);
	uv = v_mult(cyl->dir, v_dot(cyl->dir, oc));
	rec->normal = v_normalize(v_less(oc, uv));
	if (v_dot(ray->dir, rec->normal) > 0)
	{
	//	rec->normal.x = -rec->normal.x;
	//	rec->normal.y = -rec->normal.y;
	//	rec->normal.z = -rec->normal.z;
		rec->inside = 1;
	}
}

int	hit_cylindre(t_cylindre *cyl, t_ray *ray, double *min_max, t_record *rec)
{
	t_vecteur	oc;
	double		tab[5];

	oc = v_cross(cyl->dir, ray->dir);
	tab[0] = v_dot(oc, oc);
	tab[1] = 2 * v_dot(oc, v_cross(cyl->dir, v_less(ray->ori, cyl->base)));
	oc = v_cross(cyl->dir, v_less(ray->ori, cyl->base));
	tab[2] = v_dot(oc, oc) - cyl->radius * cyl->radius;
	tab[3] = tab[1] * tab[1] - 4 * tab[0] * tab[2];
	if (tab[3] > 0)
	{
		tab[4] = (-1 * tab[1] - sqrt(tab[3])) / (2 * tab[0]);
		if (tab[4] < min_max[1] && tab[4] > min_max[0])
		{
			cyl_rec(ray, tab[4], cyl, rec);
			return (1);
		}
		tab[4] = (-1 * tab[1] + sqrt(tab[3])) / (2 * tab[0]);
		if (tab[4] < min_max[1] && tab[4] > min_max[0])
		{
			cyl_rec(ray, tab[4], cyl, rec);
			return (1);
		}
	}
	return (0);
}
