/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   object_set.c                                       :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/06/09 16:49:07 by scornaz           #+#    #+#             */
/*   Updated: 2018/06/09 16:53:25 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "object.h"
#include "object_texture.h"
#include <SDL_image.h>

void		square_set(t_obj *obj, char *type, double *val)
{
	if (ft_strequ(type, "sphere"))
	{
		obj->obj.sphere->radius = *val;
		obj->obj.sphere->radius2 = *val * *val;
	}
	else if (ft_strequ(type, "cylinder"))
	{
		obj->obj.cylinder->radius = *val;
		obj->obj.cylinder->radius2 = *val * *val;
	}
	else if (ft_strequ(type, "fcylinder"))
	{
		obj->obj.fcylinder->radius = *val;
		obj->obj.fcylinder->radius2 = *val * *val;
	}
	else if (ft_strequ(type, "cone"))
	{
		obj->obj.cone->angle = *val;
		obj->obj.cone->angle2 = *val * *val;
		obj->obj.cone->tan_angle = tan(*val / 2);
		obj->obj.cone->tan_angle2 = 1 + tan(*val / 2) * tan(*val / 2);
	}
	else if (ft_strequ(type, "fcone"))
		obj->obj.fcone2->angle = *val;
}

static void	colori(t_obj *obj, char *type, void *val)
{
	if (ft_strequ(type, "light"))
	{
		vec3f_cpy(&(obj->emission_color), val);
		vec3f_set(&(obj->surface_color), 0, 0, 0);
	}
	else
	{
		vec3f_cpy(&(obj->surface_color), val);
		vec3f_set(&(obj->emission_color), 0, 0, 0);
	}
}

void		object_set2(t_obj *obj, char *prop, char *type, void *val)
{
	if (ft_strequ(prop, "radius") || ft_strequ(prop, "angle"))
		square_set(obj, type, val);
	else if (ft_strequ(prop, "mid"))
		obj->obj.fcone2->mid = *((double*)val);
	else if (ft_strequ(prop, "height"))
	{
		if (ft_strequ(type, "fcylinder"))
			obj->obj.fcylinder->size = *((double*)val);
		else if (ft_strequ(type, "fcone"))
			obj->obj.fcone2->size = *((double*)val);
	}
	else if (ft_strequ(prop, "reflection"))
		obj->reflection = *((double*)val);
	else if (ft_strequ(prop, "transparency"))
		obj->transparency = *((double*)val);
	if (ft_strequ(prop, "color"))
		colori(obj, type, val);
	if (ft_strequ(prop, "ior"))
		obj->ior = *((double*)val);
	if (ft_strequ(prop, "size"))
		if (ft_strequ(type, "disque"))
			obj->obj.plane->size = *((double*)val);
}

static void	positi(t_obj *obj, char *type, void *val)
{
	obj->position = *((t_vec3f*)val);
	if (ft_strequ(type, "plane") || ft_strequ(type, "disque"))
		vec3f_cpy(&(obj->obj.plane->position), val);
	else if (ft_strequ(type, "cone"))
		vec3f_cpy(&(obj->obj.cone->tip_position), val);
	else if (ft_strequ(type, "shpere") || ft_strequ(type, "light"))
		vec3f_cpy(&(obj->obj.sphere->center), val);
	else if (ft_strequ(type, "fcone"))
		vec3f_cpy(&(obj->obj.fcone2->tip_position), val);
	else if (ft_strequ(type, "fcylinder"))
		vec3f_cpy(&(obj->obj.fcylinder->base), val);
	else if (ft_strequ(type, "cylinder"))
		vec3f_cpy(&(obj->obj.cylinder->position), val);
	else
		vec3f_cpy(&(obj->obj.sphere->center), val);
}

void		object_set(t_obj *obj, char *prop, char *type, void *val)
{
	if (ft_strequ(prop, "position"))
		positi(obj, type, val);
	else if (ft_strequ(prop, "texture"))
		texture_set(obj, val);
	else if (ft_strequ(prop, "axis"))
	{
		vec3f_normalize(val);
		if (ft_strequ(type, "cylinder"))
			vec3f_cpy(&(obj->obj.cylinder->axis), val);
		else if (ft_strequ(type, "fcone"))
			vec3f_cpy(&(obj->obj.fcone2->axis), val);
		else if (ft_strequ(type, "fcylinder"))
			vec3f_cpy(&(obj->obj.fcylinder->dir), val);
		else if (ft_strequ(type, "plane") || ft_strequ(type, "disque"))
			vec3f_cpy(&(obj->obj.plane->axis), val);
		else
			vec3f_cpy(&(obj->obj.cone->axis), val);
	}
	else
		object_set2(obj, prop, type, val);
}
