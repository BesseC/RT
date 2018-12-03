/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   libft.h                                            :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/01/24 11:40:21 by scornaz           #+#    #+#             */
/*   Updated: 2018/04/14 12:30:15 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef LIBFT_H
# define LIBFT_H

# include <string.h>
# include <stdlib.h>
# include <stdarg.h>
# include "array/array.h"
# include "maths/ft_maths.h"
# include "unicode/unicode.h"
# include "gnl/get_next_line.h"
# include "list/lst_includes.h"
# include "string/ft_string.h"
# include "mem/mem.h"
# include "print/print.h"
# include "numbers/numbers.h"

# define EXIT_ERROR		exit(-1)
# define MALLOC_ERROR	write(2, "malloc error", 12); EXIT_ERROR;

int				ft_notneg(int value);
size_t			ft_nbrsize(int nbr);
char			*ft_swapchar(char *s, int ind1, int ind2);
char			*ft_rotate(const char *s, int rotation);
void			ft_print_2d(char **matrix);
void			ft_print_len(int len, int *data);

#endif
