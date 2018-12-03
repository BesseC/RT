/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   object.h                                           :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/06/09 16:49:19 by scornaz           #+#    #+#             */
/*   Updated: 2018/06/09 19:56:04 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef OBJECT_H
# define OBJECT_H

# include "vec3f.h"
# include "ray.h"
# include "resolve_quadratics.h"
# include "sphere.h"
# include "cone.h"
# include "cylinder.h"
# include "cyl_fini.h"
# include "cone2_fini.h"
# include "plane.h"
# include "libft.h"
# include "printf.h"
# include <SDL.h>
# include "object_texture.h"

typedef struct		s_obj{
	char			*tag;
	union			u_type{
		t_sphere	*sphere;
		t_cone		*cone;
		t_plane		*plane;
		t_cylinder	*cylinder;
		t_fcylinder	*fcylinder;
		t_fcone2	*fcone2;
	}				obj;
	double			transparency;
	double			reflection;
	t_vec3f			surface_color;
	t_vec3f			emission_color;
	t_vec3f			position;
	t_obj_texture	texture;
	t_vec3f			txtr_ang;
	t_vec3f			txtr_pos;
	double			ior;
}					t_obj;

t_obj				object_new(char *type, char *parent);
void				object_del(void *obj, size_t size);
void				object_set(t_obj *obj, char *type, char *prop, void *val);
void				object_print(t_obj *obj);
int					object_intersect(t_obj *obj, t_ray *hit, double *t0);
void				object_normale(t_obj *obj, t_ray *hit);
t_obj				**configure(char *config_file);
int					texture_set(t_obj *obj, char *val);
void				object_texture(t_obj *obj, t_ray *hit);
t_vec3f				object_get_texture_pixel(double x, double y, t_obj_texture \
		texture);
void				print_texture(t_obj_texture *texture);

#endif
