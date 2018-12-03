/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   array.h                                            :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/01/22 11:27:52 by scornaz           #+#    #+#             */
/*   Updated: 2018/01/22 14:40:46 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef ARRAY_H
# define ARRAY_H

# include "../libft.h"

typedef struct	s_array
{
	void		*mem;
	size_t		type_len;
	unsigned	space;
	unsigned	cursor;
	int			size;
}				t_array;

t_array			*new_array();
void			free_array(t_array *array);
void			array_add(t_array *array, void *val, unsigned len);

#endif
