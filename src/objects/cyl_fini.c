/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   cyl_fini.c                                         :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: cbesse <marvin@42.fr>                      +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/05/11 19:29:22 by cbesse            #+#    #+#             */
/*   Updated: 2018/05/26 17:31:16 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include <stdio.h>
#include "cyl_fini.h"

t_fcylinder	*fcylindre_new(t_vec3f position, t_vec3f dir,
		double radius, double size)
{
	t_fcylinder	*fcyl;

	fcyl = (t_fcylinder *)ft_memalloc(sizeof(t_fcylinder));
	fcyl->dir = dir;
	fcyl->base = position;
	fcyl->radius = radius;
	fcyl->radius2 = radius * radius;
	fcyl->size = size;
	return (fcyl);
}

void		fcylindre_print(t_fcylinder *cylinder)
{
	ft_printf("radius: ");
	ft_printf("\ndir:");
	vec3f_print(&cylinder->dir);
}
