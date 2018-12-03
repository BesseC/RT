/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   sphere.h                                           :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/04/02 14:38:30 by scornaz           #+#    #+#             */
/*   Updated: 2018/06/09 17:35:16 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef SPHERE_H
# define SPHERE_H

# include "vec3f.h"
# include "ray.h"

typedef struct	s_sphere {
	t_vec3f		center;
	double		radius;
	double		radius2;
}				t_sphere;

t_sphere		*sphere_new(t_vec3f center, double radius);
int				sphere_intersect(t_sphere *sphere, t_ray *hit, double *res, \
		t_vec3f *pos);

void			sphere_print(t_sphere *sphere);
void			sphere_normale(t_sphere *sphere, t_ray *hit);
void			sphere_texture(t_ray *hit);

#endif
