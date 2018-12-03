/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   libft.h                                            :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/01/24 11:40:21 by scornaz           #+#    #+#             */
/*   Updated: 2018/01/24 12:31:19 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef LIBFT_H
# define LIBFT_H

# include <string.h>
# include <stdlib.h>
# include <stdarg.h>
# include "array/array.h"
# include "unicode/unicode.h"
# include "list/includes.h"
# include "gnl/get_next_line.h"
# include "list/includes.h"
# include "string/ft_string.h"
# include "mem/mem.h"
# include "print/print.h"

int				ft_abs(int value);
int				ft_notneg(int value);
size_t			ft_nbrsize(int nbr);
int				ft_atoi(const char *str);
char			*ft_itoa(int n);
unsigned int	ft_fact(unsigned int nb);
char			**ft_all_perms(char *s);
char			*ft_swapchar(char *s, int ind1, int ind2);
char			*ft_rotate(const char *s, int rotation);
void			ft_print_2d(char **matrix);
void			ft_print_len(int len, int *data);
int				ft_max(int one, int two);
int				ft_min(int one, int two);

#endif
