/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   colors.c                                           :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: acourtin <acourtin@student.42.fr>          +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/05/08 11:14:56 by acourtin          #+#    #+#             */
/*   Updated: 2018/06/08 18:34:01 by acourtin         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "rtv1.h"
#include "colors.h"

static void		apply_filter(t_clr *c, t_cfilter f)
{
	if (f == SEPIA)
		*c = (t_clr){(c->r * .393) + (c->g * .769) + (c->b * .189),
			(c->r * .349) + (c->g * .686) + (c->b * .168),
			(c->r * .272) + (c->g * .534) + (c->b * .131)};
	else if (f == GRAYSCALE)
	{
		*c = (t_clr){(c->r + c->g + c->b) / 3,
			(c->r + c->g + c->b) / 3, (c->r + c->g + c->b) / 3};
		c->r > 80 ? c->r *= 1.5 : 1;
		c->g > 80 ? c->g *= 1.5 : 1;
		c->b > 80 ? c->b *= 1.5 : 1;
	}
	else if (f == NEGATIVE)
		*c = (t_clr){255 - c->r, 255 - c->g, 255 - c->b};
	else if (f == HELL)
	{
		*c = (t_clr){c->r * 1.5, c->g * .8, c->b * .6};
		c->r = c->r > 255 ? 255 : c->r;
	}
	else if (f == CONTRAST)
		*c = (t_clr){128 + (c->r - 128) * 1.2, 128 + (c->g - 128) * 1.2,
			128 + (c->b - 128) * 1.2};
	else
		apply_filter2(c, f);
}

void			get_lumas(t_mclr *c, t_render_opts *opts, int i)
{
	c->lce = determine_luma(&c->ce);
	if ((int)(i - opts->width) >= 0)
		destr2(opts->pixels[(int)(i - opts->width)], &c->up, &c->okup, &c->lup);
	else
		c->okup = 0;
	if ((int)(i + opts->width) < opts->width * opts->height)
		destr2(opts->pixels[(int)(i + opts->width)], &c->dn, &c->okdn, &c->ldn);
	else
		c->okdn = 0;
	if (i - 1 >= 0)
		destr2(opts->pixels[i - 1], &c->le, &c->okle, &c->lle);
	else
		c->okle = 0;
	if (i + 1 < opts->width * opts->height)
		destr2(opts->pixels[i + 1], &c->ri, &c->okri, &c->lri);
	else
		c->okri = 0;
	get_lumas2(c, opts, i);
}

static void		apply_fxaa(t_mclr *c, t_render_opts *opts, int i)
{
	t_lumas			l;

	get_lumas(c, opts, i);
	l.luma_min = MIN(c->lce, MIN(MIN(c->ldn, c->lup), MIN(c->lle, c->lri)));
	l.luma_max = MAX(c->lce, MAX(MAX(c->ldn, c->lup), MAX(c->lle, c->lri)));
	l.luma_range = l.luma_max - l.luma_min;
	if (l.luma_range < MAX(EDGE_THRESHOLD_MIN, l.luma_max * EDGE_THRESHOLD_MAX))
	{
		c->res.r = c->ce.r;
		c->res.g = c->ce.g;
		c->res.b = c->ce.b;
	}
	else
		calculate_fxaa(&l, c);
	c->res.r = c->res.r > 255 ? 255 : c->res.r;
	c->res.g = c->res.g > 255 ? 255 : c->res.g;
	c->res.b = c->res.b > 255 ? 255 : c->res.b;
	c->res.r = c->res.r < 0 ? 0 : c->res.r;
	c->res.g = c->res.g < 0 ? 0 : c->res.g;
	c->res.b = c->res.b < 0 ? 0 : c->res.b;
}

static void		ready_fxaa(t_render_opts *opts)
{
	int			i;
	t_mclr		c;

	i = -1;
	while (++i < opts->width * opts->height)
	{
		destr(opts->pixels[i], &c.ce);
		if (i % (int)opts->width == opts->width - 1 || i < opts->width || \
			i % (int)opts->width == 0 || \
			i / (int)opts->width == (int)opts->height - 1)
			c.res = (t_clr){c.ce.r, c.ce.g, c.ce.b};
		else
			apply_fxaa(&c, opts, i);
		opts->rended[i] = restr(c.res.r, c.res.g, c.res.b);
	}
}

void			change_colors(t_render_opts *opts, t_cfilter f)
{
	int			i;
	t_clr		c;

	if (f == NONE)
		ft_memcpy(opts->rended, opts->pixels, sizeof(int) * (opts->width \
			* opts->height));
	else if (f == FXAA && opts->it < 1)
		ready_fxaa(opts);
	else if ((f == CELLSHADING || f == PENCIL) && opts->it < 1)
		ready_cellshading(opts, f);
	else
	{
		i = -1;
		while (++i < opts->height * opts->width)
		{
			destr(opts->pixels[i], &c);
			if (opts->it < 1)
				apply_filter(&c, f);
			opts->rended[i] = restr(c.r, c.g, c.b);
		}
	}
}
