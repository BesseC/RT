/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   change_types.c                                     :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/01/24 11:24:39 by scornaz           #+#    #+#             */
/*   Updated: 2018/05/09 17:34:25 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "parser_printf.h"
#include "libft.h"

void		change_type(t_flags *flags)
{
	if (ft_strchr(flags->type, 'U'))
	{
		flags->type[0] = 'l';
		flags->type[1] = 'u';
	}
	else if (ft_strchr(flags->type, 'O'))
	{
		flags->type[0] = 'l';
		flags->type[1] = 'o';
	}
	else if (ft_strchr(flags->type, 'D'))
	{
		flags->type[0] = 'l';
		flags->type[1] = 'd';
	}
	else if (ft_strchr(flags->type, 'p'))
	{
		flags->hash = 1;
		flags->type[0] = 'l';
		flags->type[1] = 'l';
		flags->type[2] = 'p';
	}
}
