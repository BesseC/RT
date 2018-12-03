/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_lstfind.c                                       :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/04/01 16:08:55 by scornaz           #+#    #+#             */
/*   Updated: 2018/04/14 12:27:53 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "ft_string.h"

t_list	*ft_lstfind(t_list *el, int (*f)(t_list *el, void *cmp), void *cmp)
{
	while (el)
	{
		if (f(el, cmp))
			return (el);
		el = el->next;
	}
	return (0);
}
