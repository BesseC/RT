/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_putendl_fd.c                                    :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2017/11/09 12:21:27 by scornaz           #+#    #+#             */
/*   Updated: 2017/11/09 14:43:00 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include <unistd.h>

void	ft_putendl_fd(char const *s, int fd)
{
	unsigned int	len;

	len = 0;
	while (s[len])
		len++;
	write(fd, s, len);
	write(fd, "\n", 1);
}
