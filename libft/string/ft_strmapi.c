/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_strmapi.c                                       :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2017/11/09 13:49:17 by scornaz           #+#    #+#             */
/*   Updated: 2017/11/12 19:23:31 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include <stdlib.h>

char	*ft_strmapi(char const *s, char (*f)(unsigned int, char))
{
	char			*res;
	unsigned int	i;

	if (!s)
		return (NULL);
	if (!f)
		return ((char*)s);
	i = 0;
	while (s[i++])
		;
	if (!(res = (char *)malloc(sizeof(char) * i)))
		return (NULL);
	i = 0;
	while (*s)
	{
		res[i] = f(i, *s++);
		i++;
	}
	res[i] = '\0';
	return (res);
}
