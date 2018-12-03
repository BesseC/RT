/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   level.c                                            :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/01/26 15:13:02 by scornaz           #+#    #+#             */
/*   Updated: 2018/04/01 16:04:41 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "btree.h"

int			btree_level_count(t_btree *root)
{
	int	left_nb;
	int	right_nb;

	left_nb = 0;
	right_nb = 0;
	if (!root)
		return (0);
	if (root->right)
	{
		++right_nb;
		right_nb += btree_level_count(root->right);
	}
	if (root->left)
	{
		++left_nb;
		left_nb += btree_level_count(root->left);
	}
	return (right_nb > left_nb ? right_nb : left_nb);
}

static void	tree_level(t_btree *node, t_io args, void *data,
					void (*apply)(void*, t_io, void*))
{
	if (!node)
		apply(0, args, data);
	else if (!args.i)
		apply(node->data, args, data);
	else
	{
		--args.i;
		if (args.k)
			++args.k;
		tree_level(node->left, args, data, apply);
		if (node->left)
			++args.k;
		tree_level(node->right, args, data, apply);
	}
}

void		btree_apply_level(t_btree *root, void *data,
							void (*apply)(void *item, t_io, void*))
{
	int		height;
	int		i;

	i = 0;
	height = btree_level_count(root);
	while (i <= height)
	{
		tree_level(root, (t_io){i, i, 0, height, 0}, data, apply);
		++i;
	}
}
