/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   vec3f.h                                            :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/04/14 16:56:50 by scornaz           #+#    #+#             */
/*   Updated: 2018/06/09 17:34:01 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef VEC3F_H
# define VEC3F_H

# include <stdlib.h>
# include <stdio.h>
# include <math.h>
# include "printf.h"

typedef struct	s_vec2d{
	double		x;
	double		y;
}				t_vec2d;

typedef struct	s_vec3f {
	double		x;
	double		y;
	double		z;
}				t_vec3f;

typedef struct	s_vec3i
{
	int			x;
	int			y;
	int			z;
}				t_vec3i;

typedef struct	s_33mat{
	t_vec3f		up;
	t_vec3f		right;
	t_vec3f		forward;
	t_vec3f		transpose;
}				t_33mat;

t_vec3f			*vec3f_new_unit (double x);
t_vec3f			*vec3f_new (double x, double y, double z);
void			vec3f_set (t_vec3f *vec3f, double x, double y, double z);
t_vec3f			*vec3f_normalize (t_vec3f *a);
t_vec3f			*vec3f_mul_unit (t_vec3f *a, double f);
t_vec3f			*vec3f_mul (t_vec3f *a, t_vec3f *b);
t_vec3f			*vec3f_sub (t_vec3f *a, t_vec3f *b);
t_vec3f			*vec3f_add (t_vec3f *a, t_vec3f *b);
t_vec3f			*vec3f_negate (t_vec3f *a);
t_vec3f			vec3f_cross(t_vec3f *a, t_vec3f *b);
double			vec3f_dot (t_vec3f *a, t_vec3f *b);
t_33mat			matrix_new(t_vec3f from, t_vec3f to, t_vec3f tmp);
t_vec3f			matrix_mul(t_33mat matrix, t_vec3f vector);
void			matrix_print(t_33mat *matrix);
double			length2 (t_vec3f *a);
double			length (t_vec3f *a);
void			vec3f_print (t_vec3f *a);
void			vec3f_mul_unit2 (t_vec3f *a, double f);
void			vec3f_mul2 (t_vec3f *a, t_vec3f *b);
void			vec3f_add2 (t_vec3f *a, t_vec3f *b);
void			vec3f_add_unit2 (t_vec3f *a, double b);
void			vec3f_sub2 (t_vec3f *a, t_vec3f *b);
void			vec3f_cpy (t_vec3f *a, t_vec3f *b);
void			vec3f_normalize2(t_vec3f *a);

#endif
