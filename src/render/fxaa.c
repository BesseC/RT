/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   fxaa.c                                             :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: acourtin <acourtin@student.42.fr>          +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/05/22 13:42:15 by acourtin          #+#    #+#             */
/*   Updated: 2018/06/08 19:05:55 by acourtin         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "rtv1.h"
#include "colors.h"

void			get_lumas2(t_mclr *c, t_render_opts *opts, int i)
{
	if (opts->pixels[(int)(i - opts->width - 1)])
		destr2(opts->pixels[(int)(i - opts->width - 1)], &c->ul, &c->okul, \
			&c->lul);
	else
		c->okul = 0;
	if (opts->pixels[(int)(i - opts->width + 1)])
		destr2(opts->pixels[(int)(i - opts->width + 1)], &c->ur, &c->okur, \
			&c->lur);
	else
		c->okur = 0;
	if (opts->pixels[(int)(i + opts->width - 1)])
		destr2(opts->pixels[(int)(i + opts->width - 1)], &c->dl, &c->okdl, \
			&c->ldl);
	else
		c->okdl = 0;
	if (opts->pixels[(int)(i + opts->width + 1)])
		destr2(opts->pixels[(int)(i + opts->width + 1)], &c->dr, &c->okdr, \
			&c->ldr);
	else
		c->okdr = 0;
}

static void		blur_edges(t_lumas *l, t_mclr *c, int allver, int allhor)
{
	if (l->edge_horizontal < l->edge_vertical)
		c->res = (t_clr){(c->ce.r + c->up.r * 6 * c->okup + c->dn.r * 6 \
			* c->okdn + c->ul.r * 4 * c->okul + c->ur.r * 4 * c->okur \
			+ c->dl.r * 4 * c->okdl + c->dr.r * 4 * c->okdr) / (allver + 1), \
			(c->ce.g + c->up.g * 6 * c->okup + c->dn.g * 6 * c->okdn + c->ul.g \
			* 4 * c->okul + c->ur.g * 4 * c->okur + c->dl.g * 4 * c->okdl \
			+ c->dr.g * 4 * c->okdr) / (allver + 1), (c->ce.b + c->up.b * 6 \
			* c->okup + c->dn.b * 6 * c->okdn + c->ul.b * 4 * c->okul \
			+ c->ur.b * 4 * c->okur + c->dl.b * 4 * c->okdl + c->dr.b * 4 \
			* c->okdr) / (allver + 1)};
	else
		c->res = (t_clr){(c->ce.r + c->le.r * 6 * c->okle + c->ri.r * 6 \
			* c->okri + c->ul.r * 4 * c->okul + c->ur.r * 4 * c->okur \
			+ c->dl.r * 4 * c->okdl + c->dr.r * 4 * c->okdr) / (allhor + 1), \
			(c->ce.g + c->le.g * 6 * c->okle + c->ri.g * 6 * c->okri + c->ul.g \
			* 4 * c->okul + c->ur.g * 4 * c->okur + c->dl.g * 4 * c->okdl \
			+ c->dr.g * 4 * c->okdr) / (allhor + 1), (c->ce.b + c->le.b * 6 \
			* c->okle + c->ri.b * 6 * c->okri + c->ul.b * 4 * c->okul \
			+ c->ur.b * 4 * c->okur + c->dl.b * 4 * c->okdl + c->dr.b * 4 \
			* c->okdr) / (allhor + 1)};
}

void			calculate_fxaa(t_lumas *l, t_mclr *c)
{
	int		allver;
	int		allhor;

	l->luma_downup = c->ldn + c->lup;
	l->luma_leftright = c->lle + c->lri;
	l->luma_leftcorners = c->ldl + c->lul;
	l->luma_downcorners = c->ldl + c->ldr;
	l->luma_rightcorners = c->ldr + c->lur;
	l->luma_upcorners = c->lur + c->lul;
	l->edge_horizontal = ABS(-2.0 * c->lle + l->luma_leftcorners) + \
		ABS(-2.0 * c->lce + l->luma_downup) * 2.0 + \
		ABS(-2.0 * c->lri + l->luma_rightcorners);
	l->edge_vertical = ABS(-2.0 * c->lup + l->luma_upcorners) + \
		ABS(-2.0 * c->lce + l->luma_leftright) * 2.0 + \
		ABS(-2.0 * c->ldn + l->luma_downcorners);
	allver = c->okup * 6 + c->okdn * 6 + c->okul * 4 + c->okur * 4 + c->okdl \
		* 4 + c->okdr * 4;
	allhor = c->okle * 6 + c->okri * 6 + c->okul * 4 + c->okur * 4 + c->okdl \
		* 4 + c->okdr * 4;
	blur_edges(l, c, allver, allhor);
}
