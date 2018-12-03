/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   rt2.h                                              :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: alerandy <alerandy@student.42.fr>          +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/05/30 14:46:48 by alerandy          #+#    #+#             */
/*   Updated: 2018/06/10 19:18:01 by cbesse           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef RT2_H
# define RT2_H

# include "rt.h"
# include "vec3f.h"
# include "object_texture.h"

typedef struct	s_vecteur
{
	double		x;
	double		y;
	double		z;
}				t_vecteur;

typedef struct	s_record
{
	double			t;
	t_vecteur		p;
	t_vecteur		normal;
	t_vecteur		color;
	int				hit_anything;
	double			ks;
	double			kt;
	int				inside;
	int				index;
	int				f;
	t_obj_texture	material;
	t_vec3f			angle;
	t_vec3f			pos;
	t_vecteur		texture;
	int				type;
	double			ior;
}				t_record;

typedef struct	s_ray2
{
	t_vecteur	ori;
	t_vecteur	dir;
}				t_ray2;

typedef struct	s_sphere2
{
	t_vecteur	center;
	double		radius;
	t_vecteur	c;
}				t_sphere2;

typedef struct	s_cylindre
{
	t_vecteur	base;
	t_vecteur	dir;
	double		radius;
	t_vecteur	color;
}				t_cylindre;

typedef struct	s_cone2
{
	t_vecteur	apex;
	t_vecteur	dir;
	double		angle;
	t_vecteur	color;
}				t_cone2;

typedef struct	s_plan
{
	t_vecteur	point;
	t_vecteur	vdir;
	t_vecteur	color;
	double		size;
}				t_plan;

typedef struct	s_fcylindre
{
	t_vecteur	base;
	t_vecteur	dir;
	double		radius;
	double		size;
	t_plan		*plan1;
	t_plan		*plan2;
}				t_fcylindre;

typedef struct	s_fcone
{
	t_vecteur	apex;
	t_vecteur	dir;
	double		angle;
	double		mid;
	double		size;
	t_plan		*plan1;
	t_plan		*plan2;
}				t_fcone;

typedef struct	s_tore
{
	t_vecteur	center;
	t_vecteur	dir;
	double		grad;
	double		prad;
}				t_tore;

typedef struct	s_formlist
{
	void			*form;
	int				type;
	int				size;
	t_vecteur		color;
	double			ks;
	double			kt;
	int				index;
	t_obj_texture	material;
	t_vec3f			angle;
	t_vec3f			pos;
	double			ior;
}				t_formlist;

typedef struct	s_scene
{
	t_formlist	*list;
	t_vecteur	*light;
	t_vecteur	camorigin;
	t_vecteur	camdir;
	int			n_light;
	int			n_obj;
	int			i;
	int			k;
}				t_scene;

typedef struct	s_equ3
{
	double				p;
	double				q;
	double				delta;
	int					k;
	_Complex double		j;
	_Complex double		j2;
}				t_equ3;

typedef struct	s_equ4
{
	double				p;
	double				q;
	double				r;
	double				y;
	_Complex double		a0;
	double				b0;
	double				a[5];
	_Complex double		x[4];
}				t_equ4;
#endif
