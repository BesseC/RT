/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   utils.c                                            :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/01/26 17:21:20 by scornaz           #+#    #+#             */
/*   Updated: 2018/04/01 16:06:47 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "btree.h"
#include <stdio.h>

t_btree			*new_btree(void *data)
{
	t_btree		*ret;

	if (!(ret = malloc(sizeof(t_btree))))
		return (0);
	ret->right = 0;
	ret->left = 0;
	ret->data = data;
	return (ret);
}

void			print(void *str, t_io args, void *opts)
{
	int level;
	int width;
	int s;
	int rank;
	int	space;

	level = args.j;
	width = ((t_point*)opts)->x;
	s = width / (1 << level);
	rank = args.k;
	space = !rank ? s / 2 : s;
	if (!rank)
		printf("\n");
	printf("%*s", space, (char*)str ? (char*)str : " ");
}

inline void		print_btree(t_btree *root)
{
	t_point	size;

	size = get_term_size();
	size.x /= 1.5;
	btree_apply_level(root, &size, print);
	printf("\n");
}
