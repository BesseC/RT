/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   parser.h                                           :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/04/11 19:29:57 by scornaz           #+#    #+#             */
/*   Updated: 2018/05/10 19:04:27 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef PARSER_H
# define PARSER_H

# include "srcs/types/errors.h"
# include "srcs/types/data.h"
# include "object.h"
# include "globals.h"
# include "array.h"

void		p(void *el, t_array *a);
void		conf_free(t_conf *conf);
void		print_objects(t_list *el);
void		print_error(void *el, t_array *array);
int			is_keyword(t_list *el, void *cmp_str);
t_list		*lex(char *buf);
void		parse(t_envir envir);
void		create_args(t_array *array, char *token,
						char type, t_envir *envir);
t_conf		*read_configuration(char *config_file, char *rules_file);
int			begin_parse(char *txt_rules, char *txt_config, t_conf *conf);

#endif
