/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_putnbr_fd.c                                     :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2017/11/09 12:19:30 by scornaz           #+#    #+#             */
/*   Updated: 2017/11/11 19:09:17 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include <unistd.h>

void	ft_putnbr_fd(int n, int fd)
{
	char	value;
	long	cast;

	cast = n;
	if (cast < 0)
	{
		write(fd, "-", 1);
		cast = -cast;
	}
	if (cast > 9)
		ft_putnbr_fd(cast / 10, fd);
	value = '0' + cast % 10;
	write(fd, &value, 1);
}
