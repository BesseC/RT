/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_strlcat.c                                       :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: cbesse <marvin@42.fr>                      +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2017/11/10 17:21:42 by cbesse            #+#    #+#             */
/*   Updated: 2017/11/10 20:47:05 by cbesse           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "libft.h"

size_t	ft_strlcat(char *dest, const char *src, size_t size)
{
	size_t	len;
	size_t	i;

	i = 0;
	while (dest[i] && i < size)
		i++;
	len = i;
	while (src[i - len] && i + 1 < size)
	{
		dest[i] = src[i - len];
		i++;
	}
	if (len < size)
		dest[i] = '\0';
	return (len + ft_strlen(src));
}
