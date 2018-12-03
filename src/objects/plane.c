/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   plane.c                                            :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/04/02 14:37:17 by scornaz           #+#    #+#             */
/*   Updated: 2018/06/07 12:13:55 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "plane.h"

t_plane			*plane_new(t_vec3f position, t_vec3f normale)
{
	t_plane		*plane;

	plane = ft_memalloc(sizeof(t_plane));
	plane->position = position;
	plane->axis = normale;
	plane->size = 0;
	return (plane);
}

void			plane_print(t_plane *plane)
{
	ft_printf("normale :");
	vec3f_print(&plane->axis);
}

int				plane_intersect(t_plane *plane, t_ray *hit, double *res)
{
	double		dist_axis;
	double		dir_axis;
	double		result;

	dir_axis = vec3f_dot(&plane->axis, &hit->raydir);
	if (dir_axis > 0)
	{
		dist_axis = vec3f_dot(&hit->distance, &plane->axis);
		result = -dist_axis / dir_axis;
		if (result >= 0)
		{
			*res = result;
			return (1);
		}
		return (0);
	}
	return (0);
}

inline void		plane_normale(t_plane *plane, t_ray *hit)
{
	vec3f_cpy(&hit->nhit, &plane->axis);
}
