/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   cone2_fini.h                                       :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: cbesse <marvin@42.fr>                      +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/05/23 13:26:45 by cbesse            #+#    #+#             */
/*   Updated: 2018/06/07 13:28:35 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef CONE2_FINI_H
# define CONE2_FINI_H

# include "vec3f/vec3f.h"

typedef struct	s_fcone2
{
	double		angle;
	t_vec3f		tip_position;
	t_vec3f		axis;
	double		size;
	double		mid;
	t_vec3i		teta;
}				t_fcone2;

t_fcone2		*fcone_new(double angle, double height,
						t_vec3f tip_position, t_vec3f axis);
void			fcone_print(t_fcone2 *cone);
#endif
