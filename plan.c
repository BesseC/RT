/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   plan.c                                             :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: cbesse <marvin@42.fr>                      +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/04/26 16:02:46 by cbesse            #+#    #+#             */
/*   Updated: 2018/04/26 16:02:48 by cbesse           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "rt.h"

int plan_ok(t_ray *ray, t_plan *plan, t_record *rec, double t)
{
  t_vecteur p;
  double ok;

  p = v_add(ray->ori, v_mult(ray->dir, t));
  ok = v_norm(v_less(plan->point, p));
	//if (ok <= plan->size)
	//printf("%f\n", ok);
  if (ok <= plan->size || plan->size == 0)
    return(1);
  return(0);
}

int	hit_plan(t_plan *plan, t_ray *ray, double *min_max, t_record *rec)
{
	t_vecteur	oc;
	double		d;
	double		temp;

	oc = v_less(ray->ori, plan->point);
	d = v_dot(ray->dir, plan->vdir);
	if (d != 0)
	{
		temp = - v_dot(oc, plan->vdir) / d;

	if (temp < min_max[1] && temp > min_max[0] && plan_ok(ray, plan, rec, temp))
	{
		rec->t = temp;
		rec->p = v_add(ray->ori, v_mult(ray->dir, rec->t));
		rec->normal = v_normalize(v_set(plan->vdir.x, plan->vdir.y,
			plan->vdir.z));
		if (v_dot(ray->dir, plan->vdir) > 0)
		{
			rec->normal.x = -rec->normal.x;
			rec->normal.y = -rec->normal.y;
			rec->normal.z = -rec->normal.z;
			rec->inside = 1;
		}
		return (1);
	}
}
	return (0);
}
