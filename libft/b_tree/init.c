/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   init.c                                             :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/01/26 10:27:07 by scornaz           #+#    #+#             */
/*   Updated: 2018/01/26 17:25:59 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "btree.h"

void		btree_apply_prefix(t_btree *root, void (*apply)(void *data))
{
	if (root)
		apply(root->data);
	if (root->left)
		btree_apply_prefix(root->left, apply);
	if (root->right)
		btree_apply_prefix(root->right, apply);
}

void		btree_apply_postfix(t_btree *root, void (*apply)(void *data))
{
	if (root)
		apply(root->data);
	if (root->right)
		btree_apply_postfix(root->right, apply);
	if (root->left)
		btree_apply_postfix(root->left, apply);
}

void		btree_apply_infix(t_btree *root, void (*apply)(void *data))
{
	if (root->left)
		btree_apply_infix(root->left, apply);
	if (root)
		apply(root->data);
	if (root->right)
		btree_apply_infix(root->right, apply);
}

void		btree_apply_left(t_btree *root, void (*apply)(void *data))
{
	if (root->left)
		btree_apply_infix(root->left, apply);
	if (root)
		apply(root->data);
}

void		btree_apply_right(t_btree *root, void (*apply)(void *data))
{
	if (root->right)
		btree_apply_infix(root->right, apply);
	if (root)
		apply(root->data);
}
