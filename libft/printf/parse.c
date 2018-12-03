/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   parse.c                                            :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/01/18 18:47:02 by scornaz           #+#    #+#             */
/*   Updated: 2018/01/25 20:03:13 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "printf.h"

int			split_type(char *type, t_num *a)
{
	if (type && ft_strlen(type) == 1)
	{
		a->modifiers = 0;
		a->type = type[0];
	}
	else if (type && (ft_strlen(type) == 2))
	{
		a->modifiers = ft_strsub(type, 0, 1);
		a->type = type[1];
	}
	else if (type && (ft_strlen(type) == 3))
	{
		a->modifiers = ft_strsub(type, 0, 2);
		a->type = type[2];
	}
	else
		return (0);
	return (1);
}

int			parse_value(void *value, t_num *a)
{
	if (ft_strchr("oO", a->type))
		a->base = 8;
	else if (ft_strchr("xXp", a->type))
		a->base = 16;
	else
		a->base = 10;
	itoas(a, value);
	if (a->type != 'c' && ft_strequ(a->value, "0")
		&& a->precision == 0)
	{
		free(a->value);
		a->value = ft_strdup("");
		if (ft_strchr("xXp", a->type))
			a->alternate = 0;
	}
	return (1);
}

static void	re_orga2(t_num *a)
{
	if (ft_strchr("diouxDIOUXp", a->type) && a->precision > 0)
		a->zero = 0;
	if (ft_strchr("ouxOUXp", a->type))
	{
		a->sign = 0;
		a->space = 0;
	}
	else if (a->type == 'c')
	{
		a->precision = 0;
		a->space = 0;
	}
}

void		re_orga(t_num *a)
{
	re_orga2(a);
	a->str_len = ft_strchr("cC", a->type) ? 1 : ft_strlen(a->value);
	if (a->type != 'c' && ft_strequ("0", a->value) && ft_strchr("xX", a->type))
		a->alternate = 0;
	if (ft_strchr("sS", a->type))
	{
		a->str_len = (a->precision == -1 || a->precision > a->str_len) ?
			a->str_len : a->precision;
		a->precision = 0;
	}
	a->count = a->str_len;
	a->precision = ft_notneg(a->precision - a->str_len);
	if (a->alternate && ft_strchr("oO", a->type) && a->value[0] != '0')
		a->precision = a->precision ? a->precision : 1;
	a->padding = ft_notneg(ft_abs(a->padding) - (a->str_len + a->precision));
	a->padding = ft_notneg(a->padding - (a->sign != 0 || a->space));
	if (a->alternate && ft_strchr("xXp", a->type))
		a->padding = ft_notneg(a->padding - 2);
	if (a->zero && a->left)
	{
		a->precision = a->padding;
		a->padding = 0;
	}
}
