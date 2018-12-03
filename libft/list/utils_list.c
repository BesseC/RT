/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   utils_list.c                                       :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/01/21 16:50:02 by scornaz           #+#    #+#             */
/*   Updated: 2018/04/14 12:28:13 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include <unistd.h>
#include "ft_string.h"

static void	print(t_list *el)
{
	write(1, el->content, ft_strlen(el->content));
	write(1, " ", 1);
}

void		ft_lstprint(t_list *el)
{
	while (el && el->content)
	{
		print(el);
		el = el->next;
	}
}
