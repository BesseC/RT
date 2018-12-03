/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_putfloat.c                                      :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: acourtin <acourtin@student.42.fr>          +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/05/03 14:18:40 by acourtin          #+#    #+#             */
/*   Updated: 2018/06/11 10:05:56 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "libft.h"

void		ft_putfloat_fd(float f, int precis, int fd)
{
	int		a;
	int		i;

	a = 10;
	(int)f == 0 && f < 0 ? ft_putchar_fd('-', fd) : 0;
	ft_putnbr_fd((int)f, fd);
	ft_putchar_fd('.', fd);
	f < 0 ? f = -f : 0;
	f -= (int)f;
	i = 0;
	while (++i < precis)
	{
		(int)(f * a) == 0 ? ft_putchar_fd('0', fd) : 0;
		a *= 10;
	}
	f *= a;
	ft_putnbr_fd((int)f, fd);
}

void		ft_putfloat(float f, int precis)
{
	ft_putfloat_fd(f, precis, 0);
}
