/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   root.c                                             :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/01/21 12:27:05 by scornaz           #+#    #+#             */
/*   Updated: 2018/01/24 11:35:57 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "ft_maths.h"

double	ft_root(double nb, double exp)
{
	double	i;
	double	res;
	int		precision;

	i = 1;
	res = 0;
	precision = 7;
	if (!exp)
		return (0);
	while (precision)
	{
		if (ft_pow(res, exp) > nb)
		{
			res -= i;
			i /= 10;
			--precision;
		}
		res += i;
	}
	return (res);
}

double	ft_sqrt(double nb)
{
	double	i;
	double	res;
	int		precision;

	i = 1;
	res = 0;
	precision = 7;
	while (precision)
	{
		if (res * res > nb)
		{
			res -= i;
			i /= 10;
			--precision;
		}
		res += i;
	}
	return (res);
}

double	ft_div(double a, double b)
{
	double	tmp;
	double	res;
	double	i;
	int		remainders[(int)b];

	tmp = 0;
	res = 0;
	i = 1;
	ft_bzero(remainders, (int)b);
	while (tmp <= a)
	{
		if (tmp == a)
			break ;
		if (tmp + b > a)
		{
			remainders[(int)tmp] = 1;
			b /= 10;
			i /= 10;
		}
		tmp += b;
		res += i;
	}
	return (res);
}
