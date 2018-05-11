/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_atoi.c                                          :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: cbesse <marvin@42.fr>                      +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2017/11/09 15:12:31 by cbesse            #+#    #+#             */
/*   Updated: 2017/11/10 13:04:07 by cbesse           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "libft.h"

int	ft_atoi(const char *nptr)
{
	int	signe;
	int	nb;
	int	i;

	i = 0;
	nb = 0;
	signe = 1;
	while (nptr[i] == ' ' || (nptr[i] >= 9 && nptr[i] <= 13) ||
			(unsigned char)nptr[i] == 133 || (unsigned char)nptr[i] == 160)
		i++;
	if (nptr[i] == '-' || nptr[i] == '+')
		if (nptr[i++] == '-')
			signe = -1;
	while (nptr[i] >= '0' && nptr[i] <= '9')
	{
		nb = 10 * nb;
		nb = nb + (nptr[i] - 48);
		i++;
	}
	return (nb * signe);
}
