/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   get_next_line.h                                    :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2017/11/15 10:53:07 by scornaz           #+#    #+#             */
/*   Updated: 2017/12/31 12:45:57 by simdax           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef GET_NEXT_LINE_H
# define GET_NEXT_LINE_H

# define BUFF_SIZE		64

# include <sys/types.h>
# include <limits.h>
# include <stdio.h>
# include "../libft.h"
# include "unistd.h"
# include <fcntl.h>

int			get_next_line(int fd, char **line);

#endif
