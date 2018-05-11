/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_memchr.c                                        :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: cbesse <marvin@42.fr>                      +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2017/11/09 12:30:45 by cbesse            #+#    #+#             */
/*   Updated: 2017/11/13 14:58:15 by cbesse           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "libft.h"

void	*ft_memchr(const void *s, int c, size_t n)
{
	unsigned char	*pt;
	size_t			i;

	i = 0;
	pt = (unsigned char *)s;
	while (i < n)
	{
		if (pt[i] == (unsigned char)c)
			return ((void *)s + i);
		else
			i++;
	}
	return (NULL);
}
