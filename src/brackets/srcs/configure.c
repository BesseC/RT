/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   configure.c                                        :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/04/25 15:55:16 by scornaz           #+#    #+#             */
/*   Updated: 2018/05/25 15:15:05 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "parser.h"
#include "globals.h"
#include "printf.h"
#include "get_next_line.h"
#include "conf.h"

t_obj			**to_array(t_list *o, t_array *count)
{
	t_obj	**objects;
	t_obj	**copy;
	int		size;
	int		*c;

	size = ft_lstsize(o) + count->cursor;
	ft_printf("size orig %d\n", ft_lstsize(o));
	objects = ft_memalloc(sizeof(t_obj*) * (size + 2));
	copy = objects;
	array_reverse(count);
	array_reduce(count, mins);
	array_reduce_index(count, plus);
	c = count->mem;
	ft_lstiter3(o, cpy, &(t_bof){&copy, (int**)(&count->mem)});
	free(c);
	free(count);
	objects[size] = 0;
	objects[size + 1] = 0;
	return (objects);
}

static void		free_conf(t_conf **conf)
{
	ft_lstdel(&(*conf)->tmp_objects, object_del);
	array_free((*conf)->num);
	free(*conf);
	*conf = 0;
}

t_conf			*read_configuration(char *config_file, char *rules_file)
{
	char		*txt_rules;
	char		*txt_config;
	t_conf		*conf;

	ft_printf("\n\e[36m****\nParsing file: %s\n****\e[0m\n\n", config_file);
	conf = ft_memalloc(sizeof(t_conf));
	conf->tmp_objects = 0;
	conf->objects = 0;
	conf->globals = (t_globals){640, 480, {0, 0, 0}, {0, 0, -1}};
	txt_rules = get_file_content(rules_file);
	txt_config = get_file_comment(config_file, '#');
	ft_printf("%s\n", txt_config);
	if (!(begin_parse(txt_rules, txt_config, conf)))
	{
		globals_print(&conf->globals);
		conf->objects = to_array(conf->tmp_objects, conf->num);
		print_obj(conf->objects);
	}
	else
		free_conf(&conf);
	free(txt_rules);
	free(txt_config);
	return (conf);
}
