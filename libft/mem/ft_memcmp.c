/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   memcmp.c                                           :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2017/11/08 19:58:28 by scornaz           #+#    #+#             */
/*   Updated: 2017/11/12 14:12:03 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include <string.h>

int	ft_memcmp(void *s1, void *s2, size_t n)
{
	unsigned char	*cpy_s1;
	unsigned char	*cpy_s2;

	if (n == 0)
		return (0);
	cpy_s1 = (unsigned char*)s1;
	cpy_s2 = (unsigned char*)s2;
	while (n-- > 1 && *cpy_s1 == *cpy_s2)
	{
		cpy_s1++;
		cpy_s2++;
	}
	return (*cpy_s1 - *cpy_s2);
}
