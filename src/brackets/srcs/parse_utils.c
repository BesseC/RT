/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   parse_utils.c                                      :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/04/12 11:12:36 by scornaz           #+#    #+#             */
/*   Updated: 2018/06/09 15:58:05 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "parser.h"
#include "object.h"
#include "globals.h"
#include "printf.h"

int			is_keyword(t_list *el, void *cmp_str)
{
	t_data	*content;

	content = el->content;
	if (content && content->type == 's')
		return (ft_strcmp(content->data.string, cmp_str) == 0);
	return (0);
}

void		print_objects(t_list *el)
{
	t_obj	*obj;

	ft_printf("\e[33m");
	obj = (t_obj*)el->content;
	if (obj)
		object_print(obj);
	ft_printf("\e[0m");
}

static int	check_types(char *arg)
{
	int		ret;

	ret = 0;
	while (*arg)
	{
		ret = ft_isdigit(*arg);
		++arg;
	}
	return (ret);
}

void		create_args(t_array *array, char *token,
						char type, t_envir *envir)
{
	int		ivalue;
	double	fvalue;
	char	*svalue;

	if (type != 's' && !check_types(token))
		error_new(envir, 2, token);
	else if (type == 'i')
	{
		ivalue = ft_atoi(token);
		array_add(array, &ivalue, sizeof(int));
	}
	else if (type == 'f')
	{
		fvalue = ft_atof(token);
		array_add(array, &fvalue, sizeof(double));
	}
	else if (type == 's')
	{
		svalue = token;
		array_add(array, svalue, ft_strlen(svalue));
		array_add(array, "\0", 1);
	}
}
