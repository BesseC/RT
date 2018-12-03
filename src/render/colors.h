/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   colors.h                                           :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: acourtin <acourtin@student.42.fr>          +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/05/08 11:16:40 by acourtin          #+#    #+#             */
/*   Updated: 2018/06/08 19:13:01 by acourtin         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef COLORS_H
# define COLORS_H

# include "rtv1.h"

# define MAX(a,b) (a>=b?a:b)
# define MIN(a,b) (a<=b?a:b)
# define ABS(a) (a<0?-a:a)
# define EDGE_THRESHOLD_MIN 0.0312
# define EDGE_THRESHOLD_MAX 0.050
# define CELL_THRESHOLD_MIN 0.100
# define CELL_THRESHOLD_MAX 0.300
# define PENCIL_THRESHOLD_MIN 0.0312
# define PENCIL_THRESHOLD_MAX 0.050

typedef struct		s_clr
{
	int				r;
	int				g;
	int				b;
}					t_clr;

typedef struct		s_mclr
{
	t_clr			ce;
	t_clr			up;
	t_clr			dn;
	t_clr			le;
	t_clr			ri;
	t_clr			ul;
	t_clr			ur;
	t_clr			dl;
	t_clr			dr;
	t_clr			res;
	float			lce;
	float			lup;
	float			ldn;
	float			lle;
	float			lri;
	float			lul;
	float			lur;
	float			ldl;
	float			ldr;
	int				okup;
	int				okdn;
	int				okle;
	int				okri;
	int				okul;
	int				okur;
	int				okdl;
	int				okdr;
}					t_mclr;

typedef struct		s_lumas
{
	float luma_min;
	float luma_max;
	float luma_range;
	float luma_downup;
	float luma_leftright;
	float edge_horizontal;
	float edge_vertical;
	float luma_leftcorners;
	float luma_downcorners;
	float luma_rightcorners;
	float luma_upcorners;
}					t_lumas;

typedef enum		e_cfilter
{
	NONE,
	SEPIA,
	GRAYSCALE,
	HELL,
	CONTRAST,
	BLUEISH,
	OLDWEST,
	NEGATIVE,
	FXAA,
	CELLSHADING,
	PENCIL
}					t_cfilter;

void				destr(const int cl, t_clr *c);
void				destr2(const int cl, t_clr *c, int *ok, float *luma);
int					restr(int r, int g, int b);
float				determine_luma(t_clr *c);
void				calculate_fxaa(t_lumas *l, t_mclr *c);
void				apply_filter2(t_clr *c, t_cfilter f);

#endif
