/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_bigstrcmp.c                                     :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/01/26 19:31:30 by scornaz           #+#    #+#             */
/*   Updated: 2018/01/26 19:45:51 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include <stdarg.h>
#include "libft.h"

int		ft_bigstrcmp(char *str, int len, ...)
{
	va_list	args;

	va_start(args, len);
	while (len)
	{
		if (!(ft_strcmp(str, va_arg(args, char*))))
			return (1);
		--len;
	}
	va_end(args);
	return (0);
}
