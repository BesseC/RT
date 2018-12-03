/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   cone.c                                             :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/04/14 16:58:43 by scornaz           #+#    #+#             */
/*   Updated: 2018/06/10 12:00:59 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "cone.h"

t_cone		*cone_new(double angle, double height,
						t_vec3f tip_position, t_vec3f axis)
{
	t_cone	*cone;

	cone = ft_memalloc(sizeof(t_cone));
	cone->angle = angle;
	cone->angle2 = angle * angle;
	cone->tan_angle = tan(angle / 2);
	cone->tan_angle2 = 1 + cone->tan_angle * cone->tan_angle;
	cone->height = height;
	cone->tip_position = tip_position;
	cone->axis = axis;
	return (cone);
}

int			cone_intersect(t_cone *cone, t_ray *hit, double *res, t_vec3f *pos)
{
	t_vec3f	co;
	t_vec3f	equation;

	vec3f_cpy(&co, &hit->rayorig);
	vec3f_sub2(&co, pos);
	equation = (t_vec3f){
		vec3f_dot(&hit->raydir, &cone->axis) *
		vec3f_dot(&hit->raydir, &cone->axis) - cone->angle2,
		2 * (vec3f_dot(&hit->raydir, &cone->axis) *
				vec3f_dot(&co, &cone->axis) - vec3f_dot(&hit->raydir, &co) *
				cone->angle2),
		vec3f_dot(&co, &cone->axis) *
		vec3f_dot(&co, &cone->axis) - vec3f_dot(&co, &co) * cone->angle2
	};
	return (resolve_quadratic(equation, res));
}

void		cone_normale(t_cone *cone, t_ray *hit)
{
	t_vec3f	axis;
	t_vec3f	hit_cpy;

	hit_cpy = hit->phit;
	vec3f_sub2(&hit_cpy, &cone->tip_position);
	axis = vec3f_cross(&hit_cpy, &cone->axis);
	axis = vec3f_cross(&axis, &hit_cpy);
	vec3f_cpy(&hit->nhit, &axis);
}

void		cone_print(t_cone *cone)
{
	ft_printf("position : ");
	vec3f_print(&cone->tip_position);
	ft_printf("axis : ");
	vec3f_print(&cone->axis);
	ft_printf("angle : %g\n", cone->angle);
	ft_printf("height : %g\n", cone->height);
}
