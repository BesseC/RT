/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_sprintf.c                                       :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/01/24 12:33:21 by scornaz           #+#    #+#             */
/*   Updated: 2018/04/14 17:14:51 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "printf.h"

static char		*sprint(char *io, t_prout prout)
{
	int		tmp;
	char	**cpy;
	t_num	*cpy_nums;
	t_array	*buffer;

	buffer = array_new(sizeof(char), 1024);
	cpy = prout.str;
	cpy_nums = prout.nums;
	while (prout.len--)
		print_and_free(&(prout.nums), &(prout.str), buffer);
	if (prout.str && *(prout.str))
	{
		tmp = ft_strlen(*(prout.str));
		array_add(buffer, *(prout.str), tmp);
		if (*(prout.str) != prout.last)
			free(*(prout.str));
	}
	free(cpy);
	free(cpy_nums);
	ft_memcpy(io, buffer->mem, buffer->cursor);
	io[buffer->cursor] = 0;
	array_free(buffer);
	return (io);
}

char			*ft_sprintf(char *io, const char *str, ...)
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
	return (sprint(io, (t_prout){args.stock, args.nums, args.nb_args, str}));
}
