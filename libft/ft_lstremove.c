/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_lstremove.c                                     :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: cbesse <marvin@42.fr>                      +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2017/11/28 17:07:17 by cbesse            #+#    #+#             */
/*   Updated: 2017/11/28 17:25:14 by cbesse           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "libft.h"

static	void	ft_niquetoi(t_list **tmp)
{
	free((*tmp)->content);
	free(*tmp);
}

void			ft_lstremove(t_list **begin_list, int data_ref)
{
	t_list *tmp;
	t_list *n;

	if (*begin_list == NULL)
		return ;
	while (*begin_list && (*begin_list)->content_size == (size_t)data_ref)
	{
		tmp = *begin_list;
		*begin_list = (*begin_list)->next;
		if ((*begin_list) == NULL)
			return ;
		ft_niquetoi(&tmp);
	}
	n = *begin_list;
	while (n->next && n)
	{
		if (n->next->content_size == (size_t)data_ref)
		{
			tmp = n->next;
			n->next = tmp->next;
			ft_niquetoi(&tmp);
		}
		else if (n->next)
			n = n->next;
	}
}
