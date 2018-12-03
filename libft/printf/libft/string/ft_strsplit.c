/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_strsplit.c                                      :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2017/11/09 14:19:54 by scornaz           #+#    #+#             */
/*   Updated: 2017/11/12 19:41:42 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "libft.h"

static size_t	calc_size(const char *str, char c)
{
	size_t	res;

	res = 0;
	while (*str)
	{
		if (*str != c && (!*(str + 1) || *(str + 1) == c))
			res++;
		str++;
	}
	return (res);
}

static char		*copy(const char *s, char c)
{
	int		i;
	char	*cpy;

	i = 0;
	while (s[i] && s[i] != c)
		i++;
	if (!(cpy = (char*)malloc(sizeof(char) * i)))
		return (NULL);
	cpy[i] = '\0';
	while (--i >= 0)
		cpy[i] = s[i];
	return (cpy);
}

char			**ft_strsplit(char const *s, char c)
{
	char	**res;
	size_t	size;

	if (!s)
		return (NULL);
	size = (calc_size((char*)s, c));
	if (!(res = (char**)malloc(sizeof(char*) * (size + 1))))
		return (NULL);
	while (*s)
	{
		if (*s && *s != c)
		{
			*res++ = copy(s, c);
			while (*s && *s != c)
				s++;
		}
		else
			s++;
	}
	*res = NULL;
	return (res - size);
}
