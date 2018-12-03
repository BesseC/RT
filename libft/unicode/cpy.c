/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   cpy.c                                              :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/01/25 14:49:16 by scornaz           #+#    #+#             */
/*   Updated: 2018/01/25 21:06:46 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "libft.h"

static size_t	size(int *str)
{
	size_t	ret;

	ret = 0;
	while (*str)
	{
		++ret;
		++str;
	}
	return (ret);
}

char			*ft_unicpy(int *uni_str)
{
	char	*ret;
	size_t	len;

	len = size(uni_str);
	ret = malloc(sizeof(int) * len + 1);
	len = 0;
	while (*uni_str)
	{
		ret[len] = *uni_str;
		++len;
		++uni_str;
	}
	ret[len] = 0;
	return (ret);
}
