/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   io.c                                               :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/01/21 12:25:54 by scornaz           #+#    #+#             */
/*   Updated: 2018/01/24 11:30:50 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "ft_maths.h"

static double	sqr(double x)
{
	return (x * x);
}

double			mypow(double base, double power, double precision)
{
	if (power < 0)
		return (1 / mypow(base, -power, precision));
	if (power >= 10)
		return (sqr(mypow(base, power / 2, precision / 2)));
	if (power >= 1)
		return (base * mypow(base, power - 1, precision));
	if (precision >= 1)
		return (ft_sqrt(base));
	return (ft_sqrt(mypow(base, power * 2, precision * 2)));
}

double			t_pow3(double base, double power)
{
	return (mypow(base, power, .00000001));
}
