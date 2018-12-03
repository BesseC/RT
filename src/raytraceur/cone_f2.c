/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   cone_f2.c                                          :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: alerandy <alerandy@student.42.fr>          +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/05/26 16:44:16 by alerandy          #+#    #+#             */
/*   Updated: 2018/06/09 17:59:04 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "rt.h"
#include "object.h"

void	def_fcone3(t_fcone *fcone)
{
	t_vecteur	h1;
	t_vecteur	h2;

	h1 = v_mult(fcone->dir, fcone->mid + fcone->size / 2);
	h2 = v_mult(fcone->dir, fcone->mid - fcone->size / 2);
	fcone->plan1 = (t_plan *)ft_memalloc(sizeof(t_plan));
	fcone->plan2 = (t_plan *)ft_memalloc(sizeof(t_plan));
	fcone->plan1->point = v_add(fcone->apex, v_mult(fcone->dir, \
				fcone->mid + fcone->size / 2));
	fcone->plan2->point = v_add(fcone->apex, v_mult(fcone->dir, \
				fcone->mid - fcone->size / 2));
	fcone->plan1->vdir = v_set(fcone->dir.x, fcone->dir.y, fcone->dir.z);
	fcone->plan2->vdir = v_set(-fcone->dir.x, -fcone->dir.y, -fcone->dir.z);
	fcone->plan1->size = sqrt(fabs(pow(v_norm(h1), 2) - pow(v_norm(h1) / \
					cos(fcone->angle / 2), 2)));
	fcone->plan2->size = sqrt(fabs(pow(v_norm(h2), 2) - pow(v_norm(h2) / \
					cos(fcone->angle / 2), 2)));
}

void	def_fcone2(t_fcone *fcone, t_fcone2 *cone)
{
	fcone->apex.x = cone->tip_position.x;
	fcone->apex.y = cone->tip_position.y;
	fcone->apex.z = cone->tip_position.z;
	fcone->angle = cone->angle * M_PI / 180;
	fcone->dir.x = cone->axis.x;
	fcone->dir.y = cone->axis.y;
	fcone->dir.z = cone->axis.z;
	fcone->dir = v_normalize(fcone->dir);
	fcone->size = cone->size;
	fcone->mid = cone->mid;
	def_fcone3(fcone);
}

void	def_fcone(t_obj *obj, t_scene *scene)
{
	int	j;

	j = 0;
	scene->list[scene->i].form = (t_fcone *)ft_memalloc(sizeof(t_fcone));
	def_fcone2(scene->list[scene->i].form, obj->obj.fcone2);
	scene->list[scene->i].color.x = obj->surface_color.x;
	scene->list[scene->i].color.y = obj->surface_color.y;
	scene->list[scene->i].color.z = obj->surface_color.z;
	scene->list[scene->i].ks = obj->reflection;
	scene->list[scene->i].kt = obj->transparency;
	scene->list[scene->i].angle = obj->txtr_ang;
	scene->list[scene->i].pos = obj->txtr_pos;
	scene->list[scene->i].material = obj->texture;
	scene->list[scene->i].type = 6;
	scene->list[scene->i].ior = obj->ior;
	scene->list[scene->i].index = scene->i;
	scene->i++;
	while (j < scene->i)
		scene->list[j++].size = scene->i;
}
