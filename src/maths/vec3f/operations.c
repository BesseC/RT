/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   creation.c                                         :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/04/03 11:46:45 by scornaz           #+#    #+#             */
/*   Updated: 2018/04/03 12:10:12 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "vec3f.h"

void	vec3f_mul_unit2(t_vec3f *a, double f)
{
	vec3f_set(a, a->x * f, a->y * f, a->z * f);
}

void	vec3f_mul2(t_vec3f *a, t_vec3f *b)
{
	vec3f_set(a, a->x * b->x, a->y * b->y, a->z * b->z);
}

void	vec3f_add2(t_vec3f *a, t_vec3f *b)
{
	vec3f_set(a, a->x + b->x, a->y + b->y, a->z + b->z);
}

void	vec3f_add_unit2(t_vec3f *a, double b)
{
	vec3f_set(a, a->x + b, a->y + b, a->z + b);
}

void	vec3f_sub2(t_vec3f *a, t_vec3f *b)
{
	vec3f_set(a, a->x - b->x, a->y - b->y, a->z - b->z);
}
