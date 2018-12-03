/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   manipulations.c                                    :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/04/25 15:09:01 by scornaz           #+#    #+#             */
/*   Updated: 2018/05/11 12:20:45 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "mem.h"
#include "array.h"

void		array_for_each(t_array *array,
					void (*f)(void *el, t_array *array))
{
	unsigned	i;

	i = 0;
	while (i < array->cursor)
	{
		f(array->mem + array->type_len * i, array);
		++i;
	}
}

void		array_for_each2(t_array *array, void *data,
							void (*f)(void *el, void *data, t_array *array))
{
	unsigned	i;

	i = 0;
	while (i < array->cursor)
	{
		f(array->mem + array->type_len * i, data, array);
		++i;
	}
}

void		array_reverse(t_array *array)
{
	int	i;
	int	j;

	i = array->cursor - 1;
	j = 0;
	while (j < i)
	{
		ft_memswap(
			array->mem + (i * array->type_len),
			array->mem + (j * array->type_len),
			array->type_len);
		--i;
		++j;
	}
}

void		array_reduce(t_array *array, void (*f)(void *a, void *b))
{
	unsigned	i;

	i = 0;
	while (i < array->cursor - 1)
	{
		f(array->mem + array->type_len * i,
		array->mem + array->type_len * (i + 1));
		++i;
	}
}

void		array_reduce_index(t_array *array,
						void (*f)(void *a, void *b, int i))
{
	unsigned	i;

	i = 0;
	while (i < array->cursor - 1)
	{
		f(array->mem + array->type_len * i,
		array->mem + array->type_len * (i + 1), i);
		++i;
	}
}
