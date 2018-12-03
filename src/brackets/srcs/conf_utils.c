/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   conf_utils.c                                       :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/05/10 19:02:11 by scornaz           #+#    #+#             */
/*   Updated: 2018/05/10 19:11:07 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "parser.h"
#include "globals.h"
#include "printf.h"

void				print_obj(t_obj **objs)
{
	int o;

	o = 0;
	while (*objs)
	{
		ft_printf("une autre partie at %d\n", o);
		while (*objs)
		{
			object_print(*objs);
			++objs;
			++o;
		}
		++o;
		++objs;
	}
	ft_printf("\e[32mParsing OK\e[0m\n");
}

void				cpy(t_list *elem, int i, void *a)
{
	t_obj	***objects;
	int		**count;
	t_bof	*arg;

	arg = a;
	objects = arg->obj;
	count = arg->count;
	**objects = ((t_obj*)elem->content);
	(*objects)++;
	if (*count && **count == i + 1)
	{
		ft_printf("cut at %d\n", i);
		**objects = 0;
		(*objects)++;
		(*count)++;
	}
}

void				mins(void *a, void *b)
{
	*(int*)a = -(*(int*)b - *(int*)a);
}

void				plus(void *a, void *b, int i)
{
	*(int*)b = *(int*)b + *(int*)a + i;
}
