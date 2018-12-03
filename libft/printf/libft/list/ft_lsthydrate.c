/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_lsthydrate.c                                    :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2017/12/08 16:13:21 by scornaz           #+#    #+#             */
/*   Updated: 2017/12/08 16:14:28 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "libft.h"

t_list		*ft_lsthydrate(char **chaine)
{
	t_list	*head;
	t_list	*tmp;
	int		val;

	val = ft_atoi(*chaine);
	head = ft_lstnew(&val, sizeof(int));
	chaine++;
	tmp = head;
	while (*chaine)
	{
		val = ft_atoi(*chaine);
		tmp->next = ft_lstnew(&val, sizeof(int));
		tmp = tmp->next;
		chaine++;
	}
	return (head);
}
