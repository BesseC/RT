/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_itoa.c                                          :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: cbesse <marvin@42.fr>                      +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2017/11/10 18:36:24 by cbesse            #+#    #+#             */
/*   Updated: 2017/11/13 15:41:36 by cbesse           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "libft.h"

char static	*ft_signe(int l, int pos, int x, char *strr)
{
	while (l--)
	{
		strr[l] = x % 10 + '0';
		x = x / 10;
	}
	if (pos)
		strr[0] = '-';
	return (strr);
}

char		*ft_itoa(int n)
{
	int		signe;
	int		len;
	char	*str;
	int		tmp;

	if (n == -2147483648)
		return (ft_strdup("-2147483648"));
	tmp = n;
	len = 2;
	signe = 0;
	if (n < 0)
	{
		n *= -1;
		signe = 1;
	}
	while (tmp /= 10)
		len++;
	len = len + signe;
	if (!(str = (char *)malloc(sizeof(char) * len)))
		return (NULL);
	str[--len] = '\0';
	str = ft_signe(len, signe, n, str);
	return (str);
}
