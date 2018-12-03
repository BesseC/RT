/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_strsub.c                                        :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2017/11/09 14:03:10 by scornaz           #+#    #+#             */
/*   Updated: 2017/11/12 19:42:35 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include <stdlib.h>
#include <string.h>

char	*ft_strsub(char const *s, unsigned int start, size_t len)
{
	char	*res;
	size_t	cpy;

	cpy = len;
	if (!s || !(res = (char*)malloc(sizeof(char) * (len + 1))))
		return (NULL);
	s += start;
	while (cpy--)
		*res++ = *s++;
	*res = '\0';
	return (res - len);
}
