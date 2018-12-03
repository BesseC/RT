/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_lstnew.c                                        :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2017/11/11 13:31:27 by scornaz           #+#    #+#             */
/*   Updated: 2017/12/31 17:27:41 by simdax           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include <string.h>
#include "libft.h"
#include <stdlib.h>

t_list	*ft_lstnew(void const *content, size_t content_size)
{
	t_list	*new;
	void	*content_cpy;
	size_t	size;

	if (!(new = (t_list*)malloc(sizeof(t_list))))
		return (NULL);
	if (!content)
	{
		content_cpy = NULL;
		size = 0;
	}
	else
	{
		if (!(content_cpy = malloc(content_size)))
			return (NULL);
		ft_memcpy(content_cpy, (void*)content, content_size);
		size = content_size;
	}
	new->content = content_cpy;
	new->content_size = size;
	new->next = NULL;
	return (new);
}
