/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   hit.c                                              :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: cbesse <marvin@42.fr>                      +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/03/09 16:13:51 by cbesse            #+#    #+#             */
/*   Updated: 2018/04/11 14:26:33 by cbesse           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "rt.h"

void	ret_inter(t_formlist list, double *min_max, t_record *rec, int *hit)
{
	*hit = 1;
	set_min_max(min_max[0], rec->t, min_max);
	rec->ks = list.ks;
	rec->kt = list.kt;
	rec->color = v_set(list.color.x, list.color.y, list.color.z);
}

int	hit_qqch(t_formlist *list, t_ray *ray, double *min_max, t_record *rec)
{
	int	i;
	int	hit_anything;

	i = -1;
	hit_anything = 0;
	while (++i < list[0].size)
	{
		if (list[i].type == 1 && hit_sphere(list[i].form, ray, min_max, rec))
			ret_inter(list[i], min_max, rec, &hit_anything);
		if (list[i].type == 2 && hit_plan(list[i].form, ray, min_max, rec))
			ret_inter(list[i], min_max, rec, &hit_anything);
		if (list[i].type == 3 && hit_cylindre(list[i].form, ray, min_max, rec))
			ret_inter(list[i], min_max, rec, &hit_anything);
		if (list[i].type == 4 && hit_cone(list[i].form, ray, min_max, rec))
			ret_inter(list[i], min_max, rec, &hit_anything);
		if (list[i].type == 5 && hit_fcylindre(list[i].form, ray, min_max, rec))
			ret_inter(list[i], min_max, rec, &hit_anything);
		if (list[i].type == 6 && hit_fcone(list[i].form, ray, min_max, rec))
			ret_inter(list[i], min_max, rec, &hit_anything);
	}
	return (hit_anything);
}

void	shadow_ret_inter(t_formlist list, double *min_max, t_record *rec, int *hit)
{
	*hit = 1;
	set_min_max(min_max[0], rec->t, min_max);
	rec->ks = list.ks;
	rec->kt = list.kt;
	if (rec->kt != 0)
		set_min_max(rec->t, DBL_MAX, min_max);
	rec->color = v_set(list.color.x, list.color.y, list.color.z);
}

int	shadow_hit_qqch(t_formlist *list, t_ray *ray, double *min_max, t_record *rec)
{
	int	i;
	int	hit_anything;

	i = -1;
	hit_anything = 0;
	while (++i < list[0].size)
	{
		if (list[i].type == 1 && hit_sphere(list[i].form, ray, min_max, rec))
			shadow_ret_inter(list[i], min_max, rec, &hit_anything);
		if (list[i].type == 2 && hit_plan(list[i].form, ray, min_max, rec))
			shadow_ret_inter(list[i], min_max, rec, &hit_anything);
		if (list[i].type == 3 && hit_cylindre(list[i].form, ray, min_max, rec))
			shadow_ret_inter(list[i], min_max, rec, &hit_anything);
		if (list[i].type == 4 && hit_cone(list[i].form, ray, min_max, rec))
			shadow_ret_inter(list[i], min_max, rec, &hit_anything);
		if (list[i].type == 5 && hit_fcylindre(list[i].form, ray, min_max, rec))
			shadow_ret_inter(list[i], min_max, rec, &hit_anything);
		if (list[i].type == 6 && hit_fcone(list[i].form, ray, min_max, rec))
			ret_inter(list[i], min_max, rec, &hit_anything);
	}
	return (hit_anything);
}
