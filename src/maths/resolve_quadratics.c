/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   resolveQuadratics.c                                :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/04/03 12:08:22 by scornaz           #+#    #+#             */
/*   Updated: 2018/06/06 15:47:19 by cbesse           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "vec3f.h"
#include "rt.h"

double	sqrt3(double a)
{
	if (a < 0)
		return (-pow(-a, 1.0 / 3));
	return (pow(a, 1.0 / 3));
}

int		resolve_quadratic(t_vec3f equation, double *solution)
{
	double	det;
	double	t1;
	double	t2;

	det = equation.y * equation.y - 4 * equation.x * equation.z;
	if (det < BIAS)
		return (0);
	det = sqrt(det);
	t1 = (-equation.y + det) / (2 * equation.x);
	t2 = (-equation.y - det) / (2 * equation.x);
	if (t1 < BIAS && t2 < BIAS)
		return (0);
	else if (t2 > BIAS && t2 < t1)
		*solution = t2;
	else
		*solution = t1;
	return (1);
}

void	ft_equa2(t_equ4 *equa, double a, double b, double c)
{
	double	delta;

	delta = b * b - (4 * a * c);
	if (a == 0)
	{
		equa->x[0] = -c / b;
	}
	if (delta == 0)
	{
		equa->x[0] = -b / (2 * a);
	}
	if (delta > 0)
	{
		equa->x[0] = (-b - sqrt(delta)) / (2 * a);
		equa->x[1] = (-b + sqrt(delta)) / (2 * a);
	}
	if (delta < 0)
	{
		equa->x[0] = (-b - I * sqrt(-delta)) / (2 * a);
		equa->x[1] = (-b + I * sqrt(-delta)) / (2 * a);
	}
}
