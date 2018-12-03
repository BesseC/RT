/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_putnbr.c                                        :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2017/11/09 12:17:47 by scornaz           #+#    #+#             */
/*   Updated: 2017/11/11 19:39:04 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include <unistd.h>

void	ft_putnbr(int n)
{
	char	value;
	long	cast;

	cast = n;
	if (cast < 0)
	{
		write(1, "-", 1);
		cast = -cast;
	}
	if (cast > 9)
		ft_putnbr(cast / 10);
	value = ('0' + cast % 10);
	write(1, &value, 1);
}
