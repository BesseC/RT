/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   insert.c                                           :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/01/26 13:50:26 by scornaz           #+#    #+#             */
/*   Updated: 2018/01/26 17:24:05 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "btree.h"
#include "string.h"

void		btree_insert(t_btree **root, void *item, int (*cmpf)(void*, void*))
{
	t_btree		*head;

	head = *root;
	if (!head)
		*root = new_btree(item);
	else
	{
		if (cmpf(head->data, item) > 0)
			btree_insert(&(head->right), item, cmpf);
		else
			btree_insert(&(head->left), item, cmpf);
	}
}

void		*btree_search_item(t_btree *root, void *data_ref,
							int (*cmpf)(void*, void*))
{
	t_btree		*ret;

	if (!root)
		return (0);
	if (!cmpf(root->data, data_ref))
		return (root);
	if (root->left)
	{
		ret = btree_search_item(root->left, data_ref, cmpf);
		if (ret)
			return (ret);
	}
	if (root->right)
	{
		ret = btree_search_item(root->right, data_ref, cmpf);
		if (ret)
			return (ret);
	}
	return (0);
}
