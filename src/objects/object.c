/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   object.c                                           :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/06/09 16:49:39 by scornaz           #+#    #+#             */
/*   Updated: 2018/06/09 17:48:41 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "object.h"
#include "object_texture.h"

t_obj	object_new(char *type, char *parent)
{
	t_obj	new;

	(void)parent;
	ft_bzero(&new, sizeof(t_obj));
	new.tag = ft_strdup(type);
	new.emission_color = (t_vec3f){0, 0, 0};
	new.surface_color = (t_vec3f){0, 0, 0};
	new.ior = 1.1;
	new.texture = (t_obj_texture){0, TXT_NONE, 0, 0};
	if (ft_strequ(type, "light") || ft_strequ(type, "sphere"))
		new.obj.sphere = sphere_new((t_vec3f){0, 0, 0}, 0);
	if (ft_strequ(type, "cone"))
		new.obj.cone =
			cone_new(0, INFINITY, (t_vec3f){0, 0, 0}, (t_vec3f){0, 0, 0});
	if (ft_strequ(type, "plane") || ft_strequ(type, "disque"))
		new.obj.plane = plane_new((t_vec3f){0, 0, 0}, (t_vec3f){0, 0, 0});
	if (ft_strequ(type, "cylinder"))
		new.obj.cylinder = cylinder_new((t_vec3f){0, 0, 0}, (t_vec3f){0, 0, 0});
	if (ft_strequ(type, "fcylinder"))
		new.obj.fcylinder =
			fcylindre_new((t_vec3f){1, 1, 0}, (t_vec3f){0, 0, 0}, 1, 1);
	if (ft_strequ(type, "fcone"))
		new.obj.fcone2 =
			fcone_new(0, 1, (t_vec3f){0, 0, 0}, (t_vec3f){0, 0, 0});
	return (new);
}

void	object_del(void *obj, size_t size)
{
	t_obj	*o;

	(void)size;
	o = obj;
	ft_memdel((void**)&o->obj);
	ft_memdel((void**)&o->tag);
	ft_memdel((void**)&o->texture.name);
	SDL_FreeSurface(o->texture.surface);
	free(obj);
}

void	object_normale(t_obj *obj, t_ray *hit)
{
	t_vec3f	tmp;

	vec3f_cpy(&tmp, &hit->raydir);
	vec3f_mul_unit2(&tmp, hit->tnear);
	vec3f_cpy(&hit->phit, &hit->rayorig);
	vec3f_add2(&hit->phit, &tmp);
	vec3f_cpy(&hit->nhit, &hit->phit);
	if (ft_strequ(obj->tag, "sphere"))
		sphere_normale(obj->obj.sphere, hit);
	else if (ft_strequ(obj->tag, "light"))
		sphere_normale(obj->obj.sphere, hit);
	else if (ft_strequ(obj->tag, "cone"))
		cone_normale(obj->obj.cone, hit);
	else if (ft_strequ(obj->tag, "plane"))
		plane_normale(obj->obj.plane, hit);
	else if (ft_strequ(obj->tag, "cylinder"))
		cylinder_normale(obj->obj.cylinder, hit);
	vec3f_normalize(&hit->nhit);
	if (vec3f_dot(&hit->raydir, &hit->nhit) > 0)
	{
		vec3f_negate(&hit->nhit);
		hit->inside = 1;
	}
	object_texture(obj, hit);
}

int		object_intersect(t_obj *obj, t_ray *hit, double *t0)
{
	vec3f_cpy(&hit->distance, &hit->rayorig);
	vec3f_sub2(&hit->distance, &obj->position);
	if (ft_strequ(obj->tag, "sphere"))
		return (sphere_intersect(obj->obj.sphere, hit, t0, &obj->position));
	else if (ft_strequ(obj->tag, "cone"))
		return (cone_intersect(obj->obj.cone, hit, t0, &obj->position));
	else if (ft_strequ(obj->tag, "plane"))
		return (plane_intersect(obj->obj.plane, hit, t0));
	else if (ft_strequ(obj->tag, "cylinder"))
		return (cylinder_intersect(obj->obj.cylinder, hit, t0));
	else
		return (0);
}

void	object_print(t_obj *obj)
{
	ft_printf("type : %s @ %p\n", obj->tag, obj);
	print_texture(&obj->texture);
	ft_printf("transparency: %g\nreflection : %g\n", obj->transparency,
		obj->reflection);
	ft_printf("emissionColor : ");
	vec3f_print(&(obj->emission_color));
	ft_printf("surfaceColor : ");
	vec3f_print(&(obj->surface_color));
	ft_printf("position : ");
	vec3f_print(&(obj->position));
	if (ft_strequ(obj->tag, "sphere"))
		sphere_print(obj->obj.sphere);
	if (ft_strequ(obj->tag, "cone"))
		cone_print(obj->obj.cone);
	if (ft_strequ(obj->tag, "cylinder"))
		cylinder_print(obj->obj.cylinder);
	if (ft_strequ(obj->tag, "plane"))
		plane_print(obj->obj.plane);
	if (ft_strequ(obj->tag, "light"))
		sphere_print(obj->obj.sphere);
	if (ft_strequ(obj->tag, "fcylinder"))
		fcylindre_print(obj->obj.fcylinder);
	if (ft_strequ(obj->tag, "fcone"))
		fcone_print(obj->obj.fcone2);
	ft_printf("\\\\\\\\\\\n");
}
