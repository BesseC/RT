/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   parser.h                                           :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/01/18 19:14:44 by scornaz           #+#    #+#             */
/*   Updated: 2018/04/14 17:15:06 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef PARSER_PRINTF_H
# define PARSER_PRINTF_H

typedef struct	s_flags
{
	int			star;
	int			zero;
	int			plus;
	int			hash;
	int			minus;
	int			apostrophe;
	int			space;
	int			precision;
	int			width;
	char		type[3];
	int			count;
}				t_flags;

# define FLAGS		"0#+- '*"
# define TYPES		"sSpdDioOuUxXcC%"
# define MODIFIERS2	"hl"
# define MODIFIERS1	"jz"

t_flags			printf_parse(char *str);
void			change_type(t_flags *flags);

#endif
