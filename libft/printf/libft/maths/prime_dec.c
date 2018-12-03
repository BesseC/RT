/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   prime_dec.c                                        :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/01/21 12:26:42 by scornaz           #+#    #+#             */
/*   Updated: 2018/01/24 11:32:14 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "ft_maths.h"

int		round_up(double f)
{
	return ((int)f + 1);
}

int		is_prime(int nb)
{
	int sqrt;
	int i;

	i = 6;
	sqrt = round_up(ft_root(nb, 2));
	if (!(nb % 2) || !(nb % 3))
		return (0);
	while (i <= sqrt)
	{
		if (!(nb % (i - 1)) || !(nb % (i + 1)))
			return (0);
		i += 6;
	}
	return (1);
}

void	prime_factors(int nb)
{
	int sqrt;
	int i;

	i = 2;
	sqrt = round_up(ft_root(nb, 2));
	while (i <= sqrt)
	{
		while (!(nb % i))
		{
			printf("%d*", i);
			nb /= i;
		}
		++i;
	}
}
