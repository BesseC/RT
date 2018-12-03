/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_lstreduce.c                                     :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/01/21 16:45:10 by scornaz           #+#    #+#             */
/*   Updated: 2018/01/21 16:45:17 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "libft.h"

void	ft_lstreduce(t_list *list, void (*f)(void *a, t_list *b, void *opts),
					void *val, void *opts)
{
	while (list)
	{
		f(val, list, opts);
		list = list->next;
	}
}
