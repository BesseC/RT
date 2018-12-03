/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_atoi.c                                          :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/04/14 13:01:04 by scornaz           #+#    #+#             */
/*   Updated: 2018/06/11 11:18:00 by alerandy         ###   ########.fr       */
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

double			ft_atof(const char *str)
{
	double	n;
	int		i;
	int		k;
	int		signe;

	i = 0;
	n = 0;
	k = 0;
	signe = 1;
	if (!str)
		return (0);
	while (str[i] == ' ' || str[i] == '\v' || str[i] == '\t' || str[i] == '\r'
			|| str[i] == '\f' || str[i] == '\n')
		i++;
	signe = (str[i] == '-') ? -signe : signe;
	i = (str[i] == '-' || str[i] == '+') ? i + 1 : i;
	while (str[i] > 47 && str[i] < 58)
	{
		n = n * 10 + (str[i++] - 48);
		if (str[i] == '.')
			k = i++;
	}
	while (k != 0 && str[++k])
		signe = signe * 10;
	return (n / signe);
}
