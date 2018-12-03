/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   array_utils.c                                      :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/04/01 13:55:16 by scornaz           #+#    #+#             */
/*   Updated: 2018/04/25 15:26:26 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "array.h"

void		array_free3(t_array **array)
{
	array_free(*array);
	*array = NULL;
}

void		*array_find(t_array *array, int (*f)(void *el, void *data),
						void *data)
{
	unsigned	i;

	i = 0;
	while (i < array->cursor)
	{
		if (f(array->mem + array->type_len * i, data))
			return (array->mem + array->type_len * i);
		++i;
	}
	return (0);
}

void		*array_at(unsigned i, t_array *array)
{
	return (array->mem + array->type_len * i);
}
