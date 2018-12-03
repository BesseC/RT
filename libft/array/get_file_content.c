/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   get_file_content.c                                 :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/04/01 16:01:39 by scornaz           #+#    #+#             */
/*   Updated: 2018/04/14 11:38:26 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include "array.h"

char	*get_file_content(char *path)
{
	int		fd;
	int		ret;
	char	buffer[65];
	t_array	*txt;
	char	*file_content;

	txt = array_new(sizeof(char), 64);
	fd = open(path, O_RDONLY);
	if (fd)
	{
		while ((ret = read(fd, buffer, 64)) > 0)
		{
			buffer[ret] = 0;
			array_add(txt, buffer, ret);
		}
	}
	file_content = malloc(txt->cursor + 1);
	ft_memcpy(file_content, txt->mem, txt->cursor);
	file_content[txt->cursor] = 0;
	array_free(txt);
	return (file_content);
}
