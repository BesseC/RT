/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   errors.c                                           :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/06/09 16:50:24 by scornaz           #+#    #+#             */
/*   Updated: 2018/06/09 17:38:49 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "printf.h"
#include "parser_types.h"
#include "errors.h"

void	error_new(t_envir *envir, int err, void *cause)
{
	t_error	error;

	error.code = err;
	error.namespace = ft_strdup(envir->namespace ? envir->namespace : "global");
	error.parent = ft_strdup(envir->parent);
	error.problem = ft_strdup(cause);
	array_add(envir->bug, &error, 1);
}

void	error_del(void *el, t_array *array)
{
	t_error	*error;

	(void)array;
	error = el;
	free(error->namespace);
	free(error->parent);
	free(error->problem);
}

void	error_print(void *el, t_array *array)
{
	t_error			*error;
	static char		*error_msgs[] = {
		"no match", "not enough arg", "bad argument", "unknown type"
	};
	static char		*help_msgs[] = {
		"check your rule file", "check your config file",
		"check carefully what you type", "keep calm"
	};

	(void)array;
	error = el;
	ft_printf("\e[31mError %d : %s for %s in %s.\nCause may be '%s'.\n"
			"You should %s.\n\e[0m", error->code,
			error_msgs[error->code], error->namespace,
			error->parent ? "global" : error->parent,
			error->problem, help_msgs[error->code]);
}
