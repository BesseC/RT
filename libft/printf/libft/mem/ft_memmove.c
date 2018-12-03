/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   bzero.c                                            :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2017/11/08 18:00:51 by scornaz           #+#    #+#             */
/*   Updated: 2017/11/13 15:15:03 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "libft.h"

void	*ft_memmove(void *dst, const void *src, size_t n)
{
	size_t		i;
	char		*ssrc;
	char		*ddst;

	ssrc = (char*)src;
	ddst = (char*)dst;
	i = 0;
	if (!n)
		return (dst);
	if (ssrc < ddst)
		while (n--)
			ddst[n] = ssrc[n];
	else
		while (i < n)
		{
			ddst[i] = ssrc[i];
			i++;
		}
	return (dst);
}
