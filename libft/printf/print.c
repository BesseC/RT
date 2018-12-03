/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   print.c                                            :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/01/17 17:39:36 by scornaz           #+#    #+#             */
/*   Updated: 2018/04/14 11:48:28 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "printf.h"

void		print_arg(t_num *num, t_array *buffer)
{
	if (num->left && num->padding)
		print_padding(num->padding, num->zero ? '0' : ' ', buffer);
	if (num->alternate)
		print_alternate(num, buffer);
	if (num->sign || num->space)
		print_sign(num->sign, num->type, num->space, buffer);
	if (num->precision)
		print_padding(num->precision, '0', buffer);
	if (num->str_len)
		array_add(buffer, num->value, num->str_len);
	if (num->padding && !num->left)
		print_padding(num->padding, ' ', buffer);
}

void		print_alternate(t_num *num, t_array *buffer)
{
	if (num->type == 'X')
		array_add(buffer, "0X", 2);
	else if (num->type == 'x')
		array_add(buffer, "0x", 2);
	else if (num->type == 'p')
		array_add(buffer, "0x", 2);
}

void		print_padding(size_t count, char with, t_array *buffer)
{
	while (count)
	{
		array_add(buffer, &with, 1);
		--count;
	}
}

void		print_sign(int sign, char type, int space, t_array *buffer)
{
	(void)type;
	if (sign == -1)
		array_add(buffer, "-", 1);
	else if (sign == 1)
		array_add(buffer, "+", 1);
	else if (space)
		array_add(buffer, " ", 1);
}
