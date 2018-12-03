/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   data.c                                             :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/04/11 18:25:23 by scornaz           #+#    #+#             */
/*   Updated: 2018/05/22 11:39:14 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "parser.h"

t_data	new_data(char type, void *d)
{
	t_data	data;

	data.type = type;
	if (type == 'l')
		data.data.list = d;
	else if (type == 's')
		data.data.string = d;
	return (data);
}

void	del_data(void *c, size_t size)
{
	t_data	*content;

	content = c;
	(void)size;
	if (content)
	{
		if (content->type == 's')
			free(content->data.string);
		else if (content->type == 'l')
			ft_lstdel(&content->data.list, del_data);
	}
	free(content);
	content = 0;
}
