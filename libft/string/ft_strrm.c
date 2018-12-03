/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_strrm.c                                         :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2017/12/29 13:07:41 by scornaz           #+#    #+#             */
/*   Updated: 2017/12/29 13:10:47 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "libft.h"

char	*ft_strrm(char const *str, char c, size_t n)
{
	char	*cpy;

	cpy = ft_strnew(ft_strlen(str));
	while (*str)
	{
		if (n && *str == c)
			--n;
		else
		{
			*cpy = *str;
			++cpy;
		}
		++str;
	}
	*cpy = 0;
	return (cpy);
}
