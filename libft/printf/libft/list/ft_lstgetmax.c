/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_lstgetmax.c                                     :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/01/21 16:50:14 by scornaz           #+#    #+#             */
/*   Updated: 2018/01/21 16:50:39 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "includes.h"

int	ft_lstgetmax(t_list *list, int val,
				int (*f)(t_list *el, int val, void *f),
				void *opt)
{
	while (list)
	{
		val = f(list, val, opt);
		list = list->next;
	}
	return (val);
}
