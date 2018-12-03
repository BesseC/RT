/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   unicode.h                                          :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/01/24 11:40:11 by scornaz           #+#    #+#             */
/*   Updated: 2018/01/24 11:40:11 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef UNICODE_H
# define UNICODE_H

unsigned char	*unicode_to_utf8(int str);
char			*transform_utf8(int *str);
void			ft_putstr_utf8(int *str);
int				ft_putchar_utf8(int str);

#endif
