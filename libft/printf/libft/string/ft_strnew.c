/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_strnew.c                                        :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2017/11/09 13:35:18 by scornaz           #+#    #+#             */
/*   Updated: 2017/11/16 18:09:38 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include <string.h>
#include <stdlib.h>

char	*ft_strnew(size_t len)
{
	char	*res;

	if (!(res = (char*)malloc(sizeof(char) * (len + 1))))
		return (NULL);
	while (len)
	{
		res[len] = '\0';
		--len;
	}
	res[len] = '\0';
	return (res);
}
