/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   cylinder.c                                         :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/06/09 16:50:08 by scornaz           #+#    #+#             */
/*   Updated: 2018/06/10 17:54:56 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "cylinder.h"

t_cylinder	*cylinder_new(t_vec3f position, t_vec3f axis)
{
	t_cylinder	*cyl;

	cyl = ft_memalloc(sizeof(t_cylinder));
	cyl->axis = axis;
	cyl->radius = 1;
	cyl->radius2 = 1;
	(void)position;
	return (cyl);
}

int			cylinder_intersect(t_cylinder *cylinder, t_ray *hit, \
		double *res)
{
	return (resolve_quadratic((t_vec3f){
				vec3f_dot(&hit->raydir, &hit->raydir) -
					pow(vec3f_dot(&hit->raydir, &cylinder->axis), 2),
					2 * (vec3f_dot(&hit->raydir, &hit->distance) -
							vec3f_dot(&hit->raydir, &cylinder->axis) *
							vec3f_dot(&hit->distance, &cylinder->axis)),
					vec3f_dot(&hit->distance, &hit->distance) -
					pow(vec3f_dot(&hit->distance, &cylinder->axis), 2) -
					cylinder->radius2
					}, res));
}

void		cylinder_normale(t_cylinder *cylinder, t_ray *hit)
{
	t_vec3f uv;

	vec3f_sub2(&hit->nhit, &cylinder->position);
	vec3f_cpy(&uv, &cylinder->axis);
	vec3f_mul_unit2(&uv, vec3f_dot(&cylinder->axis, &hit->nhit));
	vec3f_sub2(&hit->nhit, &uv);
}

void		cylinder_texture(t_ray *hit)
{
	hit->texture.x = 0.5 + atan2(hit->nhit.z, hit->nhit.x) / M_PI * 0.5;
	hit->texture.y = asin(hit->nhit.y) / M_PI;
}

void		cylinder_print(t_cylinder *cylinder)
{
	ft_printf("radius: ");
	ft_putfloat(cylinder->radius, 3);
	ft_printf("\naxis:");
	vec3f_print(&cylinder->axis);
}
