/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_lstmap.c                                        :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2017/11/10 21:26:10 by scornaz           #+#    #+#             */
/*   Updated: 2017/11/11 12:55:14 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "libft.h"

t_list	*ft_lstmap(t_list *lst, t_list *(*f)(t_list *elem))
{
	t_list	*new_node;

	if (lst)
	{
		new_node = f(lst);
		if (lst->next)
			new_node->next = ft_lstmap(lst->next, f);
		return (new_node);
	}
	return (NULL);
}
