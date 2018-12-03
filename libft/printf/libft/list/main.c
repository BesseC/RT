/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   main.c                                             :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/01/21 16:46:51 by scornaz           #+#    #+#             */
/*   Updated: 2018/01/21 16:46:51 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "includes.h"

static t_list	*find_middle(t_list *head)
{
	t_list *half;

	half = head;
	while (head->next && head->next->next)
	{
		half = half->next;
		head = head->next->next;
	}
	return (half);
}

static void		put(t_list **src, t_list **dst)
{
	if (!(*dst))
		*dst = *src;
	else
	{
		(*dst)->next = *src;
		*dst = (*dst)->next;
	}
	*src = (*src)->next;
}

static t_list	*merge(t_list *one, t_list *two,
		int (*cmp)(t_list*, t_list*))
{
	t_list	*tmp;
	t_list	*head;

	head = 0;
	tmp = 0;
	while (1)
	{
		if (!one && !two)
			break ;
		else if (!two || (!one ? 0 : cmp(one, two)))
			put(&one, &tmp);
		else
			put(&two, &tmp);
		if (!head)
			head = tmp;
	}
	return (head);
}

t_list			*ft_lstsort(t_list *list, int (*cmp)(t_list*, t_list*))
{
	t_list	*half;
	t_list	*tmp;

	if (!list || !list->next)
		return (list);
	half = find_middle(list);
	tmp = half->next;
	half->next = NULL;
	return (merge(ft_lstsort(list, cmp), ft_lstsort(tmp, cmp), cmp));
}
