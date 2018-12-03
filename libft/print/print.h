/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   print.h                                            :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/01/24 12:11:30 by scornaz           #+#    #+#             */
/*   Updated: 2018/06/10 17:43:04 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef PRINT_H
# define PRINT_H

int				ft_isalpha(int c);
int				ft_isdigit(int c);
int				ft_isalnum(int c);
int				ft_isascii(int c);
int				ft_issep(char c);
int				ft_isprint(int c);
int				ft_toupper(int c);
int				ft_tolower(int c);
void			ft_putchar(char c);
void			ft_putstr(char const *s);
char			*ft_strrm(char const *s, char c, size_t n);
void			ft_putendl(char const *s);
void			ft_putnbr(int n);
void			ft_putfloat(float f, int precis);
void			ft_putfloat_fd(float f, int precis, int fd);
void			ft_putchar_fd(char c, int fd);
void			ft_putstr_fd(char const*s, int fd);
void			ft_putendl_fd(char const *s, int fd);
void			ft_putnbr_fd(int n, int fd);
void			ft_putstr_utf8(int *str);
int				ft_putchar_utf8(int str);

#endif
