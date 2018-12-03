/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_memalloc.c                                      :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2017/11/09 12:05:47 by scornaz           #+#    #+#             */
/*   Updated: 2018/04/01 16:12:11 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include <stdlib.h>
#include "libft.h"

void		*ft_memalloc(size_t size)
{
	void	*ptr;

	if (!(ptr = malloc(size)))
		return (NULL);
	while (size--)
		((char*)ptr)[size] = 0;
	return (ptr);
}

void		*ft_malloc(size_t size)
{
	void	*ptr;

	if (!(ptr = malloc(size)))
		MALLOC_ERROR;
	return (ptr);
}
