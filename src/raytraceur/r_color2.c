/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   couleur.c                                          :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: cbesse <marvin@42.fr>                      +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/03/30 14:02:00 by cbesse            #+#    #+#             */
/*   Updated: 2018/06/10 18:25:10 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "rt.h"

t_vecteur			r_background(t_ray2 *ray)
{
	t_vecteur	v2;
	t_vecteur	v1;
	t_vecteur	vr;
	double		t;

	v1 = v_set(1.0, 1.0, 1.0);
	v2 = v_set(0.5, 0.7, 1.0);
	t = 0.5 * (ray->dir.y + 1.0);
	vr = v_add(v_mult(v1, (1.0 - t)), v_mult(v2, t));
	return (vr);
}

t_vecteur			diffu_spec(t_vecteur light, t_record *r)
{
	t_vecteur	temp;
	t_vecteur	refrac;
	t_vecteur	diffu;
	double		shade;
	double		phong;

	temp = v_normalize(v_less(light, r->p));
	refrac = v_less(v_mult(r->normal, 2 * v_dot(r->normal, temp)), temp);
	phong = r->type == 2 ? 0 : v_dot(refrac, temp);
	shade = v_dot(r->normal, temp);
	shade = shade < 0 ? 0 : shade;
	phong = phong < 0 ? 0 : phong;
	phong = pow(phong, 16);
	diffu.x = r->color.x * (AMBIANT + (1 - AMBIANT) * shade) + phong;
	diffu.y = r->color.y * (AMBIANT + (1 - AMBIANT) * shade) + phong;
	diffu.z = r->color.z * (AMBIANT + (1 - AMBIANT) * shade) + phong;
	return (diffu);
}

t_ray2				refraction(t_ray2 *ray, t_record *r)
{
	t_ray2		refrac;
	double		t[5];

	t[0] = r->ior;
	t[1] = 1;
	t[4] = v_dot(r->normal, v_normalize(ray->dir));
	if (t[4] < 0)
		t[4] = -t[4];
	else
	{
		r->normal = v_negate(r->normal);
		t[1] = r->ior;
		t[0] = 1;
	}
	t[2] = t[1] / t[0];
	t[3] = 1 - t[2] * t[2] * (1 - t[4] * t[4]);
	if (t[3] > 0)
		refrac.dir = v_add(v_mult(ray->dir, t[2]), v_mult(r->normal, t[2] * \
					t[4] - sqrtf(t[3])));
	else
		refrac.dir = v_set(0, 0, 0);
	refrac.dir = v_normalize(refrac.dir);
	refrac.ori = v_less(r->p, v_mult(r[0].normal, EPSILON));
	return (refrac);
}

float				ft_fresnel(t_ray2 *ray, t_record *r)
{
	double tmp;
	double kr;

	tmp = -v_dot(ray->dir, r[0].normal);
	tmp = pow(1 - tmp, 3);
	kr = 1 * 0.1 + tmp * (1 - 0.1);
	kr = kr > 1 ? 1 : kr;
	return (kr);
}
