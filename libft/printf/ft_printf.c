/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_printf.c                                        :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/01/18 18:41:07 by scornaz           #+#    #+#             */
/*   Updated: 2018/04/14 17:14:44 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "array.h"
#include "printf.h"
#include <inttypes.h>

t_num			flags2print(va_list arg, t_flags flags)
{
	t_num		a;
	intmax_t	value;

	hydrate(&a, &flags);
	split_type(flags.type, &a);
	if (a.type == 'E')
	{
		va_arg(arg, void*);
		a.value = ft_strdup("");
		a.type = 's';
	}
	else if (ft_strchr("csSC", a.type))
		strings(&a, arg);
	else if (a.type == '%')
	{
		a.value = ft_strdup("%");
		a.type = 'c';
	}
	else
	{
		value = va_arg(arg, intmax_t);
		parse_value(&value, &a);
	}
	re_orga(&a);
	return (a);
}

static int		print(char **str, t_num *nums, int len, const char *last)
{
	int		tmp;
	char	**cpy;
	t_num	*cpy_nums;
	t_array	*buffer;

	buffer = array_new(sizeof(char), 1024);
	cpy = str;
	cpy_nums = nums;
	while (len--)
		print_and_free(&nums, &str, buffer);
	if (str && *str)
	{
		tmp = ft_strlen(*str);
		array_add(buffer, *str, tmp);
		if (*str != last)
			free(*str);
	}
	free(cpy);
	free(cpy_nums);
	write(1, buffer->mem, buffer->cursor);
	tmp = buffer->cursor;
	array_free(buffer);
	return (tmp);
}

int				count_percents(const char *str)
{
	int count;

	count = 0;
	while (*str)
	{
		if (*str == '%')
			count++;
		str++;
	}
	return (count);
}

void			init_args(t_args *args, const char *str)
{
	args->count = 0;
	args->nb_args = count_percents(str);
	args->nums = (t_num*)malloc(sizeof(t_num) * (args->nb_args + 1));
	args->stock = (char**)malloc(sizeof(char*) * (args->nb_args + 2));
}

int				ft_printf(const char *str, ...)
{
	va_list		arg;
	t_args		args;

	init_args(&args, str);
	if (args.nb_args)
	{
		va_start(arg, str);
		while (args.count < args.nb_args)
		{
			args.cpy = ft_strchr(str, '%');
			args.stock[args.count] = ft_strsub(str, 0, args.cpy - str);
			args.flags = printf_parse(args.cpy + 1);
			if (args.flags.star)
				args.flags.width = va_arg(arg, int);
			args.nums[args.count] = flags2print(arg, args.flags);
			if (args.cpy[args.flags.count] == '%')
				--args.nb_args;
			str = args.cpy + args.flags.count + 1;
			++args.count;
		}
		va_end(arg);
	}
	args.stock[args.count] = (char*)str;
	return (print(args.stock, args.nums, args.nb_args, str));
}
