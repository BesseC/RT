/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   insert.c                                           :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/01/26 13:54:52 by scornaz           #+#    #+#             */
/*   Updated: 2018/01/26 19:19:20 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "../btree.h"
#include "string.h"

int			mstrcmp(void *a, void *b)
{
	return (strcmp((char*)a, (char*)b));
}

int			main(void)
{
	t_btree	*head;
	t_btree	*tmp;

	head = 0;
	btree_insert(&head, "05", mstrcmp);
	btree_insert(&head, "09", mstrcmp);
	btree_insert(&head, "03", mstrcmp);
	btree_insert(&head, "18", mstrcmp);
	btree_insert(&head, "07", mstrcmp);
	btree_insert(&head, "04", mstrcmp);
	btree_insert(&head, "10", mstrcmp);
	print_btree(head);
}
