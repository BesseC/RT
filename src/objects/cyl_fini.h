/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   cyl_fini.h                                         :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: cbesse <marvin@42.fr>                      +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/05/11 19:48:57 by cbesse            #+#    #+#             */
/*   Updated: 2018/06/07 13:30:01 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef CYL_FINI_H
# define CYL_FINI_H

# include "vec3f/vec3f.h"

typedef struct	s_fcylinder
{
	t_vec3f		base;
	t_vec3f		dir;
	double		radius;
	double		radius2;
	double		size;
	t_vec3i		teta;
}				t_fcylinder;

t_fcylinder		*fcylindre_new(t_vec3f position, t_vec3f axis,
		double radius, double size);
void			fcylindre_print(t_fcylinder *fcyl);
#endif
