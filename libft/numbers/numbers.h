/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   numbers.h                                          :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/04/01 16:13:01 by scornaz           #+#    #+#             */
/*   Updated: 2018/04/01 16:13:13 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef NUMBERS_H
# define NUMBERS_H

int				ft_max(int one, int two);
int				ft_min(int one, int two);
unsigned int	ft_fact(unsigned int nb);
int				ft_atoi(const char *str);
double			ft_atof(const char *str);
char			*ft_itoa(int n);
char			**ft_all_perms(char *s);
int				ft_abs(int value);

#endif
