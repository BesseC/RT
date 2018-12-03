/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   data.h                                             :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/04/17 13:15:18 by scornaz           #+#    #+#             */
/*   Updated: 2018/04/17 16:02:23 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef DATA_H
# define DATA_H

# include "libft.h"

typedef struct	s_data{
	char		type;
	union		u_data{
		char	*string;
		t_list	*list;
	}			data;
}				t_data;

t_data			new_data(char type, void *d);
void			del_data(void *c, size_t size);
void			del_data2(void *c, size_t size);

#endif
