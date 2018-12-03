/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_lstiter.c                                       :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/01/21 16:46:01 by scornaz           #+#    #+#             */
/*   Updated: 2018/05/11 12:22:05 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "libft.h"

void	ft_lstiter(t_list *lst, void (*f)(t_list *elem))
{
	while (lst)
	{
		f(lst);
		lst = lst->next;
	}
}

void	ft_lstiter2(t_list *lst, void (*f)(t_list *elem, void *args), void *mem)
{
	while (lst)
	{
		f(lst, mem);
		lst = lst->next;
	}
}

void	ft_lstiter3(t_list *lst,
				void (*f)(t_list *elem, int i, void *args), void *mem)
{
	int	i;

	i = 0;
	while (lst)
	{
		f(lst, i, mem);
		lst = lst->next;
		++i;
	}
}
