/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   types_ca.h                                         :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/01/18 18:40:20 by scornaz           #+#    #+#             */
/*   Updated: 2018/01/23 14:38:43 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef TYPES_CA_H
# define TYPES_CA_H

typedef struct		s_num
{
	int		sign;
	int		star;
	int		alternate;
	int		space;
	int		base;
	char	type;
	char	*modifiers;
	int		padding;
	char	zero;
	int		left;
	int		precision;
	char	*value;
	int		str_len;
	size_t	count;
}					t_num;

typedef struct		s_str
{
	int			padding;
	void		*value;
}					t_str;

#endif
