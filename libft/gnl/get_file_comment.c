/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   get_text_comment.c                                 :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/04/16 17:38:56 by scornaz           #+#    #+#             */
/*   Updated: 2018/05/10 14:51:27 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include <fcntl.h>
#include <unistd.h>
#include "array.h"
#include "get_next_line.h"
#include "string.h"

char	*get_file_comment(char *path, char com_char)
{
	int		fd;
	char	*line;
	t_array	*array;
	int		i;

	array = array_new(1, 16);
	fd = open(path, O_RDONLY);
	while (get_next_line(fd, &line) > 0)
	{
		i = 0;
		while (ft_issep(line[i]))
			++i;
		if (line[i] != com_char)
		{
			array_add(array, line, ft_strlen(line));
			array_add(array, "\n", 1);
		}
		free(line);
	}
	array_add(array, "\0", 1);
	line = ft_strdup(array->mem);
	array_free(array);
	return (line);
}
