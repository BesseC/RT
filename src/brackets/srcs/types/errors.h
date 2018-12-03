/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   errors.h                                           :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/04/17 12:48:14 by scornaz           #+#    #+#             */
/*   Updated: 2018/04/17 21:54:56 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef ERRORS_H
# define ERRORS_H

# include "array.h"
# include "data.h"
# include "parser_types.h"

typedef struct	s_error {
	int		code;
	int		line;
	void	*problem;
	char	*namespace;
	char	*parent;
}				t_error;

void			error_new(t_envir *envir, int error, void *pb);
void			error_del(void *el, t_array *array);
void			error_print(void *el, t_array *array);

#endif
