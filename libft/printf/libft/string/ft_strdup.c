/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   strdup.c                                           :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2017/11/08 20:18:06 by scornaz           #+#    #+#             */
/*   Updated: 2018/01/15 15:29:10 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include <string.h>
#include <stdlib.h>

char	*ft_strdup(char *s1)
{
	char			*cpy;
	unsigned int	i;

	i = 0;
	while (s1[i++])
		;
	if (!(cpy = (char*)malloc(sizeof(char) * i)))
		return (NULL);
	i = 0;
	while (s1[i])
	{
		cpy[i] = s1[i];
		++i;
	}
	cpy[i] = 0;
	return (cpy);
}
