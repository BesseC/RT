/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   w_newobj.c                                         :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: alerandy <alerandy@student.42.fr>          +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/06/11 09:39:13 by alerandy          #+#    #+#             */
/*   Updated: 2018/06/11 09:47:12 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "rtv1.h"

void	w_obj5(t_obj *obj, int fd)
{
	if (ft_strequ(obj->tag, "fcylinder"))
	{
		ft_putstr_fd(" (axis (", fd);
		ft_putfloat_fd(obj->obj.fcylinder->dir.x, 3, fd);
		ft_putstr_fd(" ", fd);
		ft_putfloat_fd(obj->obj.fcylinder->dir.y, 3, fd);
		ft_putstr_fd(" ", fd);
		ft_putfloat_fd(obj->obj.fcylinder->dir.z, 3, fd);
		ft_putstr_fd("))", fd);
		ft_putstr_fd(" (radius (", fd);
		ft_putfloat_fd(obj->obj.fcylinder->radius, 3, fd);
		ft_putstr_fd("))", fd);
		ft_putstr_fd(" (height (", fd);
		ft_putfloat_fd(obj->obj.fcylinder->size, 3, fd);
		ft_putstr_fd("))", fd);
	}
}

void	w_obj6(t_obj *obj, int fd)
{
	if (ft_strequ(obj->tag, "fcone"))
	{
		ft_putstr_fd(" (axis (", fd);
		ft_putfloat_fd(obj->obj.fcone2->axis.x, 3, fd);
		ft_putstr_fd(" ", fd);
		ft_putfloat_fd(obj->obj.fcone2->axis.y, 3, fd);
		ft_putstr_fd(" ", fd);
		ft_putfloat_fd(obj->obj.fcone2->axis.z, 3, fd);
		ft_putstr_fd("))", fd);
		ft_putstr_fd(" (angle (", fd);
		ft_putfloat_fd(obj->obj.fcone2->angle, 3, fd);
		ft_putstr_fd("))", fd);
		ft_putstr_fd(" (height (", fd);
		ft_putfloat_fd(obj->obj.fcone2->size, 3, fd);
		ft_putstr_fd("))", fd);
		ft_putstr_fd(" (mid (", fd);
		ft_putfloat_fd(obj->obj.fcone2->mid, 3, fd);
		ft_putstr_fd("))", fd);
	}
}
