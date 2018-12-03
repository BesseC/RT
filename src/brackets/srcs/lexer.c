/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   lexer.c                                            :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/04/11 18:25:58 by scornaz           #+#    #+#             */
/*   Updated: 2018/04/17 21:32:23 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "parser.h"
#include "printf.h"

static void	putstr(char **buf, t_list **content)
{
	char const	*cpy;
	char		*name;
	t_data		node;

	cpy = *buf;
	while (**buf && !(**buf == ')' || **buf == '('))
		*buf += 1;
	node.type = 's';
	name = ft_strsub(cpy, 0, *buf - cpy);
	node.data.string = ft_strtrim(name);
	free(name);
	ft_lstaddlast(content, ft_lstnew(&node, sizeof(node)));
}

t_list		*lex_f(char **buf, int *count)
{
	t_list	*inside;
	t_data	data;

	inside = 0;
	while (**buf)
	{
		if (**buf == '(')
		{
			*count = *count + 1;
			++(*buf);
			data = new_data('l', lex_f(buf, count));
			ft_lstaddlast(&inside, ft_lstnew(&data, sizeof(t_data)));
		}
		else if (**buf == ')')
		{
			*count = *count - 1;
			++(*buf);
			return (inside);
		}
		else if (ft_issep(**buf))
			++(*buf);
		else
			putstr(buf, &inside);
	}
	return (inside);
}

t_list		*lex(char *buf)
{
	char		*cpy;
	int			count;
	t_list		*result;

	cpy = buf;
	count = 0;
	result = lex_f(&cpy, &count);
	if (count != 0)
	{
		ft_printf("\e[31mExpressions not well balanced: %d\n\e[0m", count);
		ft_lstdel(&result, del_data);
		return (0);
	}
	else
		return (result);
}
