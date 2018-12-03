/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   all_perms.c                                        :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2017/11/13 13:44:10 by scornaz           #+#    #+#             */
/*   Updated: 2017/11/13 14:21:48 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "libft.h"

static void		permute(char *mot, unsigned int size, char ***res)
{
	int i;

	i = size;
	if (size == 1)
		*(*res)++ = mot;
	else
		while (i)
			permute((mot = ft_swapchar(mot, i--, size)), size - 1, res);
}

char			**ft_all_perms(char *s)
{
	size_t			size;
	unsigned int	perm_size;
	char			**res;

	size = ft_strlen(s);
	if (!(perm_size = ft_fact(size - 1)) ||
		!(res = ((char**)malloc(sizeof(char*) * (perm_size + 1)))))
		return (NULL);
	permute(s, size - 1, &res);
	*res = NULL;
	return (res - 20);
}
