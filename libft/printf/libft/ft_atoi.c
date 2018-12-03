/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_atoi.c                                          :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2017/11/09 13:28:44 by scornaz           #+#    #+#             */
/*   Updated: 2017/12/29 15:06:54 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "libft.h"

static long		atoi_base(const char *str, int base)
{
	long		res;
	const char	*digits;

	digits = "0123456789ABCDEF";
	res = 0;
	while (*str && ft_isdigit(*str))
	{
		res = (res * base) +
			(ft_strchr(digits, *str) - digits);
		++str;
	}
	return (res);
}

int				ft_atoi(const char *str)
{
	long	res;
	int		neg;

	neg = 0;
	while (ft_issep(*str))
		str++;
	if (*str == '+' || (neg = (*str == '-')))
		str++;
	if (str[1] == 'x')
		res = atoi_base(str + 2, 16);
	else
		res = atoi_base(str, 10);
	return ((int)(neg ? -res : res));
}
