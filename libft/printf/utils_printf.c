/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   utils_printf.c                                     :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/01/18 18:52:23 by scornaz           #+#    #+#             */
/*   Updated: 2018/04/14 12:14:44 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include <stdlib.h>
#include "parser_printf.h"
#include "types_ca.h"
#include "prototypes_ca.h"
#include "libft.h"

void		hydrate(t_num *a, t_flags *flags)
{
	a->left = !flags->minus;
	a->star = flags->star;
	a->padding = flags->width;
	a->precision = flags->precision;
	a->alternate = flags->hash;
	a->sign = flags->plus;
	a->zero = flags->zero;
	a->space = flags->space;
}

int			print_and_free(t_num **nums, char ***str, t_array *buffer)
{
	size_t	len;

	len = ft_strlen(**str);
	array_add(buffer, **str, len);
	free(**str);
	++(*str);
	print_arg(*nums, buffer);
	free((*nums)->value);
	free((*nums)->modifiers);
	++(*nums);
	return (0);
}
