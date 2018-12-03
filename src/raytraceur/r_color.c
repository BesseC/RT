/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   r_color.c                                          :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: alerandy <alerandy@student.42.fr>          +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/05/30 13:08:09 by alerandy          #+#    #+#             */
/*   Updated: 2018/06/09 20:00:09 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "rt.h"

static void			vr_tocolor(t_vecteur *vr, t_record *r, int i)
{
	vr->x = vr->x * r[0].color.x;
	vr->y = vr->y * r[0].color.y;
	vr->z = vr->z * r[0].color.z;
	if (i)
	{
		vr->x = vr->x > 1 ? 1 : vr->x;
		vr->y = vr->y > 1 ? 1 : vr->y;
		vr->z = vr->z > 1 ? 1 : vr->z;
	}
}

static t_vecteur	ksone(t_record *r, t_ray2 *ray, t_scene scene, int depth)
{
	t_ray2		reflec;
	t_vecteur	vr;

	reflec.ori = v_add(r[0].p, v_mult(r[0].normal, EPSILON));
	reflec.dir = v_less(ray->dir, v_mult(r[0].normal, 2 * \
				v_dot(ray->dir, r[0].normal)));
	reflec.dir = v_normalize(reflec.dir);
	vr = r_color(&reflec, scene, depth + 1);
	vr_tocolor(&vr, r, 1);
	return (vr);
}

static t_vecteur	get_light(t_scene scene, double *min_max, t_record *r)
{
	int			i;
	t_ray2		sray;
	t_vecteur	vr;

	vr = v_set(0, 0, 0);
	i = scene.n_light;
	while (i-- > 0)
	{
		set_min_max(EPSILON, 1, min_max);
		sray.ori = v_add(r[0].p, v_mult(r[0].normal, EPSILON));
		sray.dir = v_less(scene.light[i], sray.ori);
		if (!(shadow_hit_qqch(scene.list, &sray, min_max, &r[1])))
			vr = c_shadow(scene.light, &r[0], vr, i);
		else if (r[1].kt > 0)
		{
			vr = c_shadow(scene.light, &r[0], vr, i);
			vr = v_div(vr, 1 - r[1].kt + 1);
			vr_tocolor(&vr, r, 1);
		}
	}
	return (vr);
}

static t_vecteur	krzero(t_record *r, t_ray2 *ray, t_scene scene, int depth)
{
	t_vecteur	vr;
	float		kr;
	t_vecteur	refraccolor;
	t_ray2		refrac;
	t_ray2		reflec;

	refraccolor = v_set(0, 0, 0);
	kr = ft_fresnel(ray, &(r[0]));
	if (kr < 1)
	{
		refrac = refraction(ray, &(r[0]));
		refrac.ori = v_less(r[0].p, v_mult(r[0].normal, \
					EPSILON));
		refraccolor = v_mult(r_color(&refrac, scene, \
					depth + 1), r[0].kt);
	}
	reflec.ori = v_add(r[0].p, v_mult(r[0].normal, EPSILON));
	reflec.dir = v_less(ray->dir, v_mult(r[0].normal, 2 * \
				v_dot(ray->dir, r[0].normal)));
	reflec.dir = v_normalize(reflec.dir);
	vr = v_add(v_mult(r_color(&reflec, scene, depth + 1), kr), \
			v_mult(refraccolor, (1 - kr)));
	vr_tocolor(&vr, r, 0);
	return (vr);
}

t_vecteur			r_color(t_ray2 *ray, t_scene scene, int depth)
{
	t_vecteur	vr;
	t_record	*r;
	double		*min_max;
	t_ray2		refrac;

	r = (t_record*)ft_memalloc(sizeof(t_record) * 2);
	min_max = (double *)ft_memalloc(2 * sizeof(double));
	set_min_max(0.0, DBL_MAX, min_max);
	if (hit_qqch(scene.list, ray, min_max, &r[0]))
	{
		vr = get_light(scene, min_max, r);
		if (depth < MAX_DEPTH)
		{
			if (r[0].ks == 1 && r[0].kt == 0)
				vr = ksone(r, ray, scene, depth);
			if (r[0].kt > 0 && r[0].ks != 1)
				refrac = refraction(ray, &(r[0]));
			if (r[0].kt > 0 && r[0].ks != 1)
				vr = v_mult(r_color(&refrac, scene, depth + 1), r[0].kt);
			if (r[0].kt > 0 && r[0].ks == 1)
				vr = krzero(r, ray, scene, depth);
		}
		return (libe((void **)&r, min_max, vr));
	}
	return (libe((void **)&r, min_max, r_background(ray)));
}
