/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_itoa.c                                          :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2017/11/09 12:06:55 by scornaz           #+#    #+#             */
/*   Updated: 2017/11/13 15:28:17 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "libft.h"

static size_t	calc_len(int n)
{
	size_t len;

	len = 0;
	while ((n /= 10))
		len++;
	return (len);
}

char			*ft_itoa(int n)
{
	char		*res;
	size_t		len;
	long		copy;

	len = 1;
	copy = n;
	len = calc_len(n);
	if (!(res = (char*)malloc((len += copy < 0) + 1)))
		return (NULL);
	*res = '0';
	if (!copy)
		return (res);
	if (copy < 0)
	{
		copy = -copy;
		*res = '-';
	}
	res += len;
	*res-- = '\0';
	while (copy)
	{
		*res-- = '0' + copy % 10;
		copy /= 10;
	}
	return (res + (*res != '-'));
}
