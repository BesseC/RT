/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_rotate.c                                        :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2017/11/13 13:49:50 by scornaz           #+#    #+#             */
/*   Updated: 2017/11/13 15:16:06 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "libft.h"

static int	modulo(int a, int b)
{
	int r;

	r = a % b;
	return (r < 0 ? r + b : r);
}

char		*ft_rotate(const char *s, int rotation)
{
	char	*res;
	size_t	size;
	size_t	i;

	i = 0;
	size = ft_strlen(s);
	res = ft_strnew(size);
	if (!s)
		return (NULL);
	while (i < size)
	{
		res[i] = s[modulo(i + rotation, size)];
		i++;
	}
	return (res);
}
