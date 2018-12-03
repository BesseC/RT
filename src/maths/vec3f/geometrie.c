/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   vec3f.c                                            :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/04/02 14:38:13 by scornaz           #+#    #+#             */
/*   Updated: 2018/04/03 11:47:30 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "vec3f.h"

double	length2(t_vec3f *a)
{
	return (a->x * a->x + a->y * a->y + a->z * a->z);
}

double	length(t_vec3f *a)
{
	return (sqrt(length2(a)));
}

double	vec3f_dot(t_vec3f *a, t_vec3f *b)
{
	return (a->x * b->x + a->y * b->y + a->z * b->z);
}

t_vec3f	*vec3f_negate(t_vec3f *a)
{
	a->x = -a->x;
	a->y = -a->y;
	a->z = -a->z;
	return (a);
}

t_vec3f	*vec3f_normalize(t_vec3f *a)
{
	double	nor;

	nor = length(a);
	if (nor > 0)
		vec3f_mul_unit2(a, 1 / nor);
	return (a);
}
