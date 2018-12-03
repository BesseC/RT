/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_swap.c                                          :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2017/11/13 13:43:35 by scornaz           #+#    #+#             */
/*   Updated: 2017/11/13 15:13:37 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "libft.h"

char		*ft_swapchar(char *mot, int i, int j)
{
	char	*copy;

	copy = ft_strdup(mot);
	copy[i] = mot[j];
	copy[j] = mot[i];
	return (copy);
}
