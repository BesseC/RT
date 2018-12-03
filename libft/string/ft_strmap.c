/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_strmap.c                                        :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2017/11/09 13:44:45 by scornaz           #+#    #+#             */
/*   Updated: 2017/11/12 19:12:38 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include <stdlib.h>

char	*ft_strmap(char const *s, char (*f)(char))
{
	char			*res;
	unsigned int	i;

	if (!s)
		return (NULL);
	i = 0;
	while (s[i++])
		;
	if (!(res = (char *)malloc(sizeof(char) * i)))
		return (NULL);
	i = 0;
	while (*s)
		res[i++] = f(*s++);
	res[i] = '\0';
	return (res);
}
