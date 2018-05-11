/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   Vec.c                                              :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: cbesse <marvin@42.fr>                      +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/03/01 15:08:10 by cbesse            #+#    #+#             */
/*   Updated: 2018/03/30 12:03:58 by cbesse           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "rt.h"

t_vecteur	v_mult(t_vecteur v1, double i)
{
	t_vecteur	v2;

	v2.x = v1.x * i;
	v2.y = v1.y * i;
	v2.z = v1.z * i;
	return (v2);
}

int			v_equal(t_vecteur v1, t_vecteur v2)
{
	if (v1.x == v2.x)
		if (v1.y == v2.y)
			if (v1.z == v2.z)
				return (1);
	return (0);
}

t_vecteur	v_div(t_vecteur v1, double i)
{
	t_vecteur	v2;

	v2.x = v1.x / i;
	v2.y = v1.y / i;
	v2.z = v1.z / i;
	return (v2);
}

t_vecteur	v_set(double x, double y, double z)
{
	t_vecteur	v;

	v.x = x;
	v.y = y;
	v.z = z;
	return (v);
}

t_vecteur	v_normalize(t_vecteur v)
{
	t_vecteur	vnorm;

	vnorm = v_div(v, v_norm(v));
	return (vnorm);
}
