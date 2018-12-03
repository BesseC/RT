/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   w_default.c                                        :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: alerandy <alerandy@student.42.fr>          +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/06/10 23:26:24 by alerandy          #+#    #+#             */
/*   Updated: 2018/06/11 09:34:22 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "rtv1.h"

void	w_texture(t_obj *obj, int fd)
{
	if (obj->texture.type != TXT_NONE)
	{
		ft_putstr_fd(" (texture (", fd);
		if (obj->texture.type == PATTERN)
		{
			ft_putstr_fd("pattern", fd);
			ft_putnbr_fd(obj->texture.pattern, fd);
		}
		else
			ft_putstr_fd(obj->texture.name, fd);
		ft_putstr_fd("))", fd);
	}
}

void	w_color(t_obj *obj, int fd)
{
	if (obj->surface_color.x || obj->surface_color.y || obj->surface_color.z)
	{
		ft_putstr_fd("(color (", fd);
		ft_putfloat_fd(obj->surface_color.x, 3, fd);
		ft_putstr_fd(" ", fd);
		ft_putfloat_fd(obj->surface_color.y, 3, fd);
		ft_putstr_fd(" ", fd);
		ft_putfloat_fd(obj->surface_color.z, 3, fd);
		ft_putstr_fd("))", fd);
	}
}

void	w_writingnolight(t_obj *obj, int fd)
{
	w_texture(obj, fd);
	w_color(obj, fd);
	if (obj->transparency)
	{
		ft_putstr_fd(" (transparency (", fd);
		ft_putfloat_fd(obj->transparency, 3, fd);
		ft_putstr_fd("))", fd);
	}
	if (obj->reflection)
	{
		ft_putstr_fd(" (reflection (", fd);
		ft_putfloat_fd(obj->reflection, 3, fd);
		ft_putstr_fd("))", fd);
	}
	if (obj->ior)
	{
		ft_putstr_fd(" (ior (", fd);
		ft_putfloat_fd(obj->ior, 3, fd);
		ft_putstr_fd("))", fd);
	}
}

void	w_obj(t_obj *obj, int fd)
{
	if (!ft_strequ(obj->tag, "light"))
	{
		w_writingnolight(obj, fd);
	}
	else
	{
		ft_putstr_fd("(color (", fd);
		ft_putfloat_fd(obj->emission_color.x, 3, fd);
		ft_putstr_fd(" ", fd);
		ft_putfloat_fd(obj->emission_color.y, 3, fd);
		ft_putstr_fd(" ", fd);
		ft_putfloat_fd(obj->emission_color.z, 3, fd);
		ft_putstr_fd("))", fd);
	}
}
