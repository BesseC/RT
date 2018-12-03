/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   get_next_line.c                                    :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2017/12/08 15:54:06 by scornaz           #+#    #+#             */
/*   Updated: 2018/05/10 14:47:37 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "get_next_line.h"

static int	ret_func(int ret, char **stat, char **line)
{
	if (*stat && ft_strlen(*stat) > 0)
	{
		*line = *stat;
		*stat = NULL;
		return (1);
	}
	return (ret);
}

static int	copy(char **stat, char **line)
{
	char *cpy;
	char *realloc;

	if ((realloc = ft_strchr(*stat, '\n')))
	{
		cpy = ft_strdup(realloc + 1);
		*line = (char*)ft_memalloc(sizeof(char) * (realloc - *stat + 1));
		ft_bzero(*line, realloc - *stat + 1);
		ft_memcpy(*line, *stat, (realloc - *stat));
		free(*stat);
		*stat = cpy;
		return (1);
	}
	return (0);
}

int			get_next_line(int fd, char **line)
{
	static char	*stat[MAX_FD];
	char		buf[BUFF_SIZE + 1];
	char		*realloc;
	int			ret;
	int			len;

	if (!line || fd < 0 || fd > MAX_FD)
		return (-1);
	if (stat[fd] && ft_strlen(stat[fd]) > 0 && copy(&stat[fd], line))
		return (1);
	while ((ret = read(fd, buf, BUFF_SIZE)) > 0)
	{
		buf[ret] = 0;
		len = stat[fd] ? ft_strlen(stat[fd]) : 0;
		realloc = (char*)ft_memalloc(sizeof(char) * (len + ret + 1));
		ft_bzero(realloc, len + ret + 1);
		if (stat[fd])
			ft_memcpy(realloc, stat[fd], len);
		ft_memcpy(realloc + len, buf, ret);
		free(stat[fd]);
		stat[fd] = realloc;
		if (copy(&stat[fd], line))
			return (1);
	}
	return (ret_func(ret, &stat[fd], line));
}
