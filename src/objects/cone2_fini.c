/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   cone2_fini.c                                       :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: cbesse <marvin@42.fr>                      +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/05/23 13:26:37 by cbesse            #+#    #+#             */
/*   Updated: 2018/06/07 12:13:27 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "cone2_fini.h"

t_fcone2	*fcone_new(double angle, double height,
						t_vec3f tip_position, t_vec3f axis)
{
	t_fcone2	*cone;

	cone = ft_memalloc(sizeof(t_fcone2));
	cone->angle = angle;
	cone->size = height;
	cone->tip_position = tip_position;
	cone->axis = axis;
	cone->mid = height / 2;
	return (cone);
}

void		fcone_print(t_fcone2 *cone)
{
	ft_printf("position : ");
	vec3f_print(&cone->tip_position);
	ft_printf("axis : ");
	vec3f_print(&cone->axis);
	ft_printf("angle : %g\n", cone->angle);
	ft_printf("height : %g\n", cone->size);
}
