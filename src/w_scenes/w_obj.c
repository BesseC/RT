/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   w_obj.c                                            :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: alerandy <alerandy@student.42.fr>          +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/06/10 18:18:24 by alerandy          #+#    #+#             */
/*   Updated: 2018/06/11 11:52:00 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "rtv1.h"

void	w_obj2(t_obj *obj, int fd)
{
	if (ft_strequ(obj->tag, "cone"))
	{
		if (obj->obj.cone->axis.x || obj->obj.cone->axis.y || \
				obj->obj.cone->axis.z)
		{
			ft_putstr_fd(" (axis (", fd);
			ft_putfloat_fd(obj->obj.cone->axis.x, 3, fd);
			ft_putstr_fd(" ", fd);
			ft_putfloat_fd(obj->obj.cone->axis.y, 3, fd);
			ft_putstr_fd(" ", fd);
			ft_putfloat_fd(obj->obj.cone->axis.z, 3, fd);
			ft_putstr_fd("))", fd);
		}
		if (obj->obj.cone->angle)
		{
			ft_putstr_fd(" (angle (", fd);
			ft_putfloat_fd(obj->obj.cone->angle, 3, fd);
			ft_putstr_fd("))", fd);
		}
	}
}

void	w_obj3(t_obj *obj, int fd)
{
	if (ft_strequ(obj->tag, "plane") || ft_strequ(obj->tag, "disque"))
	{
		if (obj->obj.plane->axis.x || obj->obj.plane->axis.y || \
				obj->obj.plane->axis.z)
		{
			ft_putstr_fd(" (axis (", fd);
			ft_putfloat_fd(obj->obj.plane->axis.x, 3, fd);
			ft_putstr_fd(" ", fd);
			ft_putfloat_fd(obj->obj.plane->axis.y, 3, fd);
			ft_putstr_fd(" ", fd);
			ft_putfloat_fd(obj->obj.plane->axis.z, 3, fd);
			ft_putstr_fd("))", fd);
			if (obj->obj.plane->size)
			{
				ft_putstr_fd(" (size (", fd);
				ft_putfloat_fd(obj->obj.plane->size, 3, fd);
				ft_putstr_fd("))", fd);
			}
		}
	}
}

void	w_obj4(t_obj *obj, int fd)
{
	if (ft_strequ(obj->tag, "cylinder"))
	{
		ft_putstr_fd(" (axis (", fd);
		ft_putfloat_fd(obj->obj.cylinder->axis.x, 3, fd);
		ft_putstr_fd(" ", fd);
		ft_putfloat_fd(obj->obj.cylinder->axis.y, 3, fd);
		ft_putstr_fd(" ", fd);
		ft_putfloat_fd(obj->obj.cylinder->axis.z, 3, fd);
		ft_putstr_fd("))", fd);
		ft_putstr_fd(" (radius (", fd);
		ft_putfloat_fd(obj->obj.cylinder->radius, 3, fd);
		ft_putstr_fd("))", fd);
	}
	if (ft_strequ(obj->tag, "sphere"))
	{
		if (obj->obj.sphere->radius)
		{
			ft_putstr_fd(" (radius (", fd);
			ft_putfloat_fd(obj->obj.sphere->radius, 3, fd);
			ft_putstr_fd("))", fd);
		}
	}
}
