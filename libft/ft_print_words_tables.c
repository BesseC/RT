/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_print_words_tables.c                            :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: cbesse <marvin@42.fr>                      +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2017/09/07 23:14:13 by cbesse            #+#    #+#             */
/*   Updated: 2017/11/16 19:14:02 by cbesse           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "libft.h"

void	ft_print_words_tables(char **tab)
{
	int i;
	int j;

	j = 0;
	while (tab[j])
	{
		i = 0;
		while (tab[j][i])
		{
			ft_putchar(tab[j][i]);
			i++;
		}
		ft_putchar('\n');
		j++;
	}
}
