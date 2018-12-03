/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   strlcat.c                                          :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2017/11/08 20:40:45 by scornaz           #+#    #+#             */
/*   Updated: 2017/11/16 18:11:11 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "libft.h"

size_t	ft_strlcat(char *dst, const char *src, size_t len)
{
	unsigned int	len_src;
	unsigned int	len_dst;
	int				i;

	len_src = ft_strlen(src);
	len_dst = ft_strlen(dst);
	dst += len_dst;
	i = len - len_dst - 1;
	while (*src && i > 0 && i--)
		*dst++ = *src++;
	*dst = '\0';
	return (len_src + ((len < len_dst) ? len : len_dst));
}
