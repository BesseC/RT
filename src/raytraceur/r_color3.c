/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   r_color3.c                                         :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: alerandy <alerandy@student.42.fr>          +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/06/09 19:57:21 by alerandy          #+#    #+#             */
/*   Updated: 2018/06/11 09:38:09 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "rt.h"

static void			texture(t_record *rec, t_vecteur *object_surface_color)
{
	t_vec3f			vtmp;

	if (rec->material.type == PATTERN)
	{
		*object_surface_color = (v_mult(*object_surface_color,
					pattern(rec, rec->material.pattern)));
	}
	else if (rec->material.type == ASSET)
	{
		vtmp = (object_get_texture_pixel(rec->texture.x, \
					rec->texture.y, rec->material));
		rec->color.x = vtmp.x;
		rec->color.y = vtmp.y;
		rec->color.z = vtmp.z;
	}
}

t_vecteur			c_shadow(t_vecteur *light, t_record *r, t_vecteur vr, \
		int n_light)
{
	if (r->material.type == ASSET)
	{
		texture(r, &vr);
		vr = v_add(diffu_spec(light[n_light], &r[0]), vr);
	}
	else if (r->material.type == PATTERN)
	{
		vr = v_add(diffu_spec(light[n_light], &r[0]), vr);
		texture(r, &vr);
	}
	else
		vr = v_add(diffu_spec(light[n_light], &r[0]), vr);
	vr.x = vr.x > 1 ? 1 : vr.x;
	vr.y = vr.y > 1 ? 1 : vr.y;
	vr.z = vr.z > 1 ? 1 : vr.z;
	return (vr);
}

static double		modulo(const double f)
{
	return (f - floor(f));
}

double				pattern(t_record *rec, unsigned type)
{
	float	angle;
	float	s;
	float	scales;
	float	scalet;
	float	t;

	angle = rec->angle.x * M_PI / 180;
	rec->texture.x = (1 + atan2(rec->normal.z, rec->normal.x + rec->pos.x) / \
			M_PI) * 0.5;
	rec->texture.y = acosf(rec->normal.y) / M_PI;
	s = (rec->texture.x) * cos(angle) - (rec->texture.y) * sin(angle);
	t = rec->texture.y * cos(angle) + rec->texture.x * sin(angle);
	scales = 20;
	scalet = 20;
	if (type == 1)
	{
		return (((cos(rec->texture.y * 2 * M_PI * scalet) * \
					sin(rec->texture.x * 2 * M_PI * scales) + 1) * 0.5) + 0.33);
	}
	else if (type == 2)
	{
		return (((modulo(s * scales) < 0.5) ^ \
					(modulo(t * scalet) < 0.5)) + 0.33);
	}
	return ((modulo(s * scales) < 0.5) + 0.33);
}
