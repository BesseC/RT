/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   cone.c                                             :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: cbesse <marvin@42.fr>                      +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/04/26 16:06:37 by cbesse            #+#    #+#             */
/*   Updated: 2018/04/26 16:06:40 by cbesse           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "rt.h"

void	cone_rec(t_ray *ray, double t, t_cone *cone, t_record *rec)
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
	{
		//rec->normal.x = -rec->normal.x;
		//rec->normal.y = -rec->normal.y;
		//rec->normal.z = -rec->normal.z;
		rec->inside = 1;
	}
}

double	*cone_tab(t_cone *cone, t_ray *ray)
{
	double		*tab;
	t_vecteur	oc;

	if (!(tab = (double *)malloc(sizeof(double) * 6)))
		return (NULL);
	tab[5] = tan(cone->angle / 2);
	tab[5] = tab[5] * tab[5];
	oc = v_less(ray->ori, cone->apex);
	tab[0] = v_dot(ray->dir, ray->dir) - (1 + tab[5]) *
		v_dot(ray->dir, cone->dir) * v_dot(ray->dir, cone->dir);
	tab[1] = 2 * (v_dot(ray->dir, oc) - (1 + tab[5]) *
			v_dot(ray->dir, cone->dir) * v_dot(oc, cone->dir));
	tab[2] = v_dot(oc, oc) - (1 + tab[5]) * v_dot(oc, cone->dir)
		* v_dot(oc, cone->dir);
	tab[4] = tab[1] * tab[1] - 4 * tab[0] * tab[2];
	return (tab);
}

int	hit_cone(t_cone *cone, t_ray *ray, double *min_max, t_record *rec)
{
	double	*tab;

	tab = cone_tab(cone, ray);
	if (tab[4] > 0)
	{
		tab[3] = (-1 * tab[1] - sqrt(tab[4])) / (2 * tab[0]);
		if (tab[3] < min_max[1] && tab[3] > min_max[0])
		{
			cone_rec(ray, tab[3], cone, rec);
			ft_memdel((void **)&tab);
			return (1);
		}
		tab[3] = (-1 * tab[1] + sqrt(tab[4])) / (2 * tab[0]);
		if (tab[3] < min_max[1] && tab[3] > min_max[0])
		{
			cone_rec(ray, tab[3], cone, rec);
			ft_memdel((void **)&tab);
			return (1);
		}
	}
	ft_memdel((void **)&tab);
	return (0);
}
