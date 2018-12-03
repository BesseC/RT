/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   power.c                                            :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/01/21 12:26:35 by scornaz           #+#    #+#             */
/*   Updated: 2018/01/24 11:31:12 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "ft_maths.h"

double	ft_pow(double nb, double pow)
{
	double	res;
	int		integer;
	double	doubleing_part;
	int		int_doubleing_part;
	double	add;

	integer = (int)pow;
	doubleing_part = pow - integer;
	res = 1;
	add = 1;
	if (nb == 0 || nb == 1)
		return (nb);
	if (integer >= 0)
		while (integer--)
			res *= nb;
	else
		return (1 / ft_pow(nb, -pow));
	if (doubleing_part > 0)
		add = ft_root(nb, doubleing_part);
	return (res * add);
}

double	ft_pow2(double nb, double pow)
{
	double	res;
	int		integer;
	double	doubleing_part;
	int		int_doubleing_part;
	double	add;

	integer = (int)pow;
	doubleing_part = pow - integer;
	res = 1 + doubleing_part;
	add = 1;
	if (nb == 0 || nb == 1)
		return (nb);
	if (integer > 0)
		while (integer--)
			res *= nb;
	else
		return (1 / ft_pow(nb, -pow));
	return (res * add);
}
