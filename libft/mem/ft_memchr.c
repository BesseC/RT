/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   memchr.c                                           :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2017/11/08 19:54:02 by scornaz           #+#    #+#             */
/*   Updated: 2017/11/09 17:04:37 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include <string.h>

void	*ft_memchr(void *s, int c, size_t n)
{
	while (n--)
	{
		if (*(unsigned char*)s == (unsigned char)c)
			return (s);
		s++;
	}
	return (NULL);
}
