/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_lstcpy.c                                        :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/01/11 13:59:41 by scornaz           #+#    #+#             */
/*   Updated: 2018/01/11 17:25:49 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "libft.h"

t_list	*ft_cpyrev(t_list *orig)
{
	t_list *ret;

	ret = 0;
	while (orig)
	{
		ft_lstadd(&ret, ft_lstnew(orig->content, orig->content_size));
		orig = orig->next;
	}
	return (ret);
}
