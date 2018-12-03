/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   test.c                                             :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/01/26 10:36:35 by scornaz           #+#    #+#             */
/*   Updated: 2018/01/26 19:09:13 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "../btree.h"
#include <stdio.h>

void	io(void *str, int i, int j)
{
	printf("node : %s %d %d \n", str, i, j);
}

int		main(void)
{
	t_btree *head;

	head = new_btree("0");
	head->left = new_btree("1");
	head->left->left = new_btree("3");
	head->left->right = new_btree("4");
	head->left->left->left = new_btree("7");
	head->left->left->right = new_btree("8");
	head->right = new_btree("2");
	head->right->left = new_btree("5");
	head->right->right = new_btree("6");
	head->right->right->left = new_btree("9");
	head->right->right->right = new_btree("10");
	print_btree(head);
}
