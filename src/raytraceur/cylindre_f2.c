/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   cylindre_f2.c                                      :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: alerandy <alerandy@student.42.fr>          +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/05/24 17:58:58 by alerandy          #+#    #+#             */
/*   Updated: 2018/06/09 17:59:10 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "rt.h"
#include "object.h"

void	fcylindre_def2(t_fcylindre *fcyl, t_fcylinder *cyl)
{
	fcyl->base.x = cyl->base.x;
	fcyl->base.y = cyl->base.y;
	fcyl->base.z = cyl->base.z;
	fcyl->radius = cyl->radius;
	fcyl->size = cyl->size;
	fcyl->dir.x = cyl->dir.x;
	fcyl->dir.y = cyl->dir.y;
	fcyl->dir.z = cyl->dir.z;
	fcyl->dir = v_normalize(fcyl->dir);
	fcyl->plan1 = (t_plan *)ft_memalloc(sizeof(t_plan));
	fcyl->plan2 = (t_plan *)ft_memalloc(sizeof(t_plan));
	fcyl->plan1->point = v_add(fcyl->base, v_mult(fcyl->dir, fcyl->size / 2));
	fcyl->plan2->point = v_less(fcyl->base, v_mult(fcyl->dir, fcyl->size / 2));
	fcyl->plan1->vdir = v_set(fcyl->dir.x, fcyl->dir.y, fcyl->dir.z);
	fcyl->plan2->vdir = v_set(-fcyl->dir.x, -fcyl->dir.y, -fcyl->dir.z);
	fcyl->plan1->size = fcyl->radius;
	fcyl->plan2->size = fcyl->radius;
}

void	fcylindre_def(t_obj *obj, t_scene *scene)
{
	int		j;

	j = 0;
	scene->list[scene->i].form = ft_memalloc(1 * sizeof(t_fcylindre));
	fcylindre_def2(scene->list[scene->i].form, obj->obj.fcylinder);
	scene->list[scene->i].color.x = obj->surface_color.x;
	scene->list[scene->i].color.y = obj->surface_color.y;
	scene->list[scene->i].color.z = obj->surface_color.z;
	scene->list[scene->i].ks = obj->reflection;
	scene->list[scene->i].kt = obj->transparency;
	scene->list[scene->i].angle = obj->txtr_ang;
	scene->list[scene->i].pos = obj->txtr_pos;
	scene->list[scene->i].material = obj->texture;
	scene->list[scene->i].type = 5;
	scene->list[scene->i].ior = obj->ior;
	scene->list[scene->i].index = scene->i;
	scene->i++;
	while (j < scene->i)
		scene->list[j++].size = scene->i;
}
