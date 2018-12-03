/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ray.h                                              :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/04/03 11:33:03 by scornaz           #+#    #+#             */
/*   Updated: 2018/06/09 17:35:00 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef RAY_H
# define RAY_H

# include "vec3f.h"

typedef struct	s_hit{
	double		tnear;
	t_vec3f		rayorig;
	t_vec3f		raydir;
	int			obj_index;
	t_vec3f		nhit;
	t_vec3f		phit;
	t_vec3f		color;
	t_vec3f		distance;
	int			inside;
	double		transmission;
	t_vec3f		refldir;
	t_vec3f		refraction;
	double		facingratio;
	t_vec3f		cp;
	double		max;
	t_vec2d		texture;
	int			x;
	int			y;
}				t_ray;

#endif
