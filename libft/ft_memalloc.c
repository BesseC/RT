/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_memalloc.c                                      :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: cbesse <marvin@42.fr>                      +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2017/11/10 12:03:49 by cbesse            #+#    #+#             */
/*   Updated: 2017/11/10 14:32:31 by cbesse           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "libft.h"

void	*ft_memalloc(size_t size)
{
	size_t			i;
	void			*mem;

	i = 0;
	if (!(mem = (void *)malloc(sizeof(char) * size)))
		return (NULL);
	ft_bzero(mem, size);
	return (mem);
}
