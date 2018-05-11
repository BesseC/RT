/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_strncat.c                                       :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: cbesse <marvin@42.fr>                      +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2017/11/09 14:29:35 by cbesse            #+#    #+#             */
/*   Updated: 2017/11/10 20:25:51 by cbesse           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "libft.h"

char	*ft_strncat(char *dest, const char *src, size_t n)
{
	char	*len;

	len = dest;
	while (*len)
	{
		len++;
	}
	while (*src != '\0' && n > 0)
	{
		*len = *src;
		len++;
		src++;
		n--;
	}
	*len = '\0';
	return (dest);
}
