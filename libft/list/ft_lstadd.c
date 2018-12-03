/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_lstadd.c                                        :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/01/21 16:45:52 by scornaz           #+#    #+#             */
/*   Updated: 2018/01/21 16:45:53 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "libft.h"

void	ft_lstadd(t_list **alst, t_list *new)
{
	new->next = *alst;
	*alst = new;
}

void	ft_lstaddlast(t_list **list, t_list *new)
{
	t_list *head;

	head = *list;
	if (head)
	{
		while (head->next)
			head = head->next;
		head->next = new;
	}
	else
		*list = new;
}
