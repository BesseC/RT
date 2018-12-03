/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_itoa.c                                          :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2017/11/09 12:06:55 by scornaz           #+#    #+#             */
/*   Updated: 2018/05/21 17:00:44 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "libft.h"

char	*ft_itoa(int nbr)
{
	char	*str;
	long	nb;
	long	a;
	int		sign;
	int		i;

	a = 10;
	i = 1;
	nb = (long)nbr;
	sign = nbr < 0 ? 1 : 0;
	nbr < 0 ? nb = -nb : nb;
	while (nb > a - 1 && i++)
		a *= 10;
	a /= 10;
	str = (char*)ft_memalloc(sizeof(char) * (i + sign) + 1);
	i = 0;
	sign ? str[i++] = '-' : i;
	while (a >= 1)
	{
		str[i++] = nb / a + '0';
		nb %= a;
		a /= 10;
	}
	str[i] = '\0';
	return (str);
}
