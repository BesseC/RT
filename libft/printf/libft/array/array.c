/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   array.c                                            :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/01/22 14:40:30 by scornaz           #+#    #+#             */
/*   Updated: 2018/01/25 18:41:39 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "array.h"

static void	array_realloc(t_array *array, size_t len)
{
	void	*tmp;
	size_t	new_size;

	tmp = array->mem;
	new_size = 0;
	while (new_size < len)
		new_size += array->space;
	array->size += new_size;
	array->mem = malloc(array->type_len * array->size);
	if (tmp)
		ft_memcpy(array->mem, tmp, array->type_len * array->cursor);
	free(tmp);
}

t_array		*new_array(size_t len, unsigned space)
{
	t_array *array;

	array = malloc(sizeof(*array));
	array->type_len = len;
	array->space = space;
	array->size = 0;
	array->mem = 0;
	array->cursor = 0;
	return (array);
}

void		free_array(t_array *array)
{
	free(array->mem);
	free(array);
}

void		array_add(t_array *array, void *val, unsigned len)
{
	if (array->size - array->cursor < len)
		array_realloc(array, len - (array->size - array->cursor));
	ft_memcpy(array->mem + array->type_len * array->cursor,
			val, array->type_len * len);
	array->cursor += len;
}
