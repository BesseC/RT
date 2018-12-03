/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   cone.h                                             :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/04/02 14:33:55 by scornaz           #+#    #+#             */
/*   Updated: 2018/06/06 15:58:45 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef CONE_H
# define CONE_H

# include "vec3f/vec3f.h"
# include "ray.h"
# include "resolve_quadratics.h"

typedef struct	s_cone
{
	double		angle;
	double		angle2;
	double		height;
	t_vec3f		tip_position;
	t_vec3f		axis;
	double		tan_angle;
	double		tan_angle2;
	t_vec3i		teta;
}				t_cone;

t_cone			*cone_new(double angle, double height,
						t_vec3f tip_position, t_vec3f axis);
int				cone_intersect(t_cone *cone, t_ray *hit, double *t0, \
		t_vec3f *pos);
void			cone_print(t_cone *cone);
void			cone_normale(t_cone *cone, t_ray *hit);

#endif
