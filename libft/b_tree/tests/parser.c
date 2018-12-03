/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   parser.c                                           :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/01/26 19:25:00 by scornaz           #+#    #+#             */
/*   Updated: 2018/04/01 16:07:53 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "../btree.h"
#include "libft.h"

int		cmpf(void *a, void *b)
{
	if (ft_bigstrcmp((char*)a, 4, "+", "-", "/", "*"))
		return (1);
	return (0);
}

int		main(int argc, char **argv)
{
	t_btree *tree;

	tree = 0;
	++argv;
	while (*argv)
	{
		btree_insert(&tree, *argv, cmpf);
		++argv;
	}
	print_btree(tree);
}
