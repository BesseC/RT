/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   c_obj.c                                            :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: alerandy <alerandy@student.42.fr>          +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/05/30 10:42:39 by alerandy          #+#    #+#             */
/*   Updated: 2018/06/09 17:58:40 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "rt.h"
#include "rtv1.h"

static void	cylindre_def2(t_cylinder *cyl, t_cylindre *cylindre, t_obj *obj)
{
	cylindre->base.x = obj->position.x;
	cylindre->base.y = obj->position.y;
	cylindre->base.z = obj->position.z;
	cylindre->radius = cyl->radius;
	cylindre->dir.x = cyl->axis.x;
	cylindre->dir.y = cyl->axis.y;
	cylindre->dir.z = cyl->axis.z;
	cylindre->dir = v_normalize(cylindre->dir);
}

void		cylindre_def(t_obj *obj, t_scene *scene)
{
	int j;

	j = 0;
	scene->list[scene->i].form =
		(t_cylindre *)ft_memalloc(1 * sizeof(t_cylindre));
	cylindre_def2(obj->obj.cylinder, scene->list[scene->i].form, obj);
	scene->list[scene->i].color.x = obj->surface_color.x;
	scene->list[scene->i].color.y = obj->surface_color.y;
	scene->list[scene->i].color.z = obj->surface_color.z;
	scene->list[scene->i].ks = obj->reflection;
	scene->list[scene->i].kt = obj->transparency;
	scene->list[scene->i].angle = obj->txtr_ang;
	scene->list[scene->i].pos = obj->txtr_pos;
	scene->list[scene->i].material = obj->texture;
	scene->list[scene->i].type = 3;
	scene->list[scene->i].ior = obj->ior;
	scene->list[scene->i].index = scene->i;
	scene->i++;
	while (j < scene->i)
		scene->list[j++].size = scene->i;
}

static void	cone_def2(t_cone *co, t_cone2 *cone, t_obj *obj)
{
	cone->apex.x = obj->position.x;
	cone->apex.y = obj->position.y;
	cone->apex.z = obj->position.z;
	cone->angle = co->angle;
	cone->dir.x = co->axis.x;
	cone->dir.y = co->axis.y;
	cone->dir.z = co->axis.z;
	cone->dir = v_normalize(cone->dir);
}

void		cone_def(t_obj *obj, t_scene *scene)
{
	int j;

	j = 0;
	scene->list[scene->i].form = (t_cone2 *)ft_memalloc(sizeof(t_cone2));
	cone_def2(obj->obj.cone, scene->list[scene->i].form, obj);
	scene->list[scene->i].color.x = obj->surface_color.x;
	scene->list[scene->i].color.y = obj->surface_color.y;
	scene->list[scene->i].color.z = obj->surface_color.z;
	scene->list[scene->i].ks = obj->reflection;
	scene->list[scene->i].kt = obj->transparency;
	scene->list[scene->i].angle = obj->txtr_ang;
	scene->list[scene->i].pos = obj->txtr_pos;
	scene->list[scene->i].material = obj->texture;
	scene->list[scene->i].type = 4;
	scene->list[scene->i].ior = obj->ior;
	scene->list[scene->i].index = scene->i;
	scene->i++;
	while (j < scene->i)
		scene->list[j++].size = scene->i;
}

void		light_def(t_sphere *sphere, t_scene *scene)
{
	int j;

	j = 0;
	scene->light[scene->k].x = sphere->center.x;
	scene->light[scene->k].y = sphere->center.y;
	scene->light[scene->k].z = sphere->center.z;
	scene->k++;
}
