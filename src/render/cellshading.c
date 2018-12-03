/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   cellshading.c                                      :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: acourtin <acourtin@student.42.fr>          +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/05/22 16:51:38 by acourtin          #+#    #+#             */
/*   Updated: 2018/06/08 17:28:27 by acourtin         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "rtv1.h"
#include "colors.h"

static int		cartoonize(int color)
{
	if (color > 200)
		return (255);
	else if (color > 150)
		return (200);
	return (color);
}

static void		apply_cellshading(t_mclr *c, t_render_opts *opts, int i, \
	t_cfilter f)
{
	t_lumas			l;

	get_lumas(c, opts, i);
	l.luma_min = MIN(c->lce, MIN(MIN(c->ldn, c->lup), MIN(c->lle, c->lri)));
	l.luma_max = MAX(c->lce, MAX(MAX(c->ldn, c->lup), MAX(c->lle, c->lri)));
	l.luma_range = l.luma_max - l.luma_min;
	if (f == CELLSHADING && l.luma_range < MAX(CELL_THRESHOLD_MIN, l.luma_max \
		* CELL_THRESHOLD_MAX))
		opts->rended[i] = restr(cartoonize(c->ce.r), cartoonize(c->ce.g), \
			cartoonize(c->ce.b));
	else if (f == PENCIL && l.luma_range < MAX(PENCIL_THRESHOLD_MIN, \
		l.luma_max * PENCIL_THRESHOLD_MAX))
		opts->rended[i] = restr(255, 254, 221);
	else
	{
		opts->rended[i] = 0;
		if (c->okup)
			opts->rended[(int)(i - opts->width)] = 0;
		if (c->okdn)
			opts->rended[(int)(i + opts->width)] = 0;
		if (c->okle)
			opts->rended[i - 1] = 0;
		if (c->okri)
			opts->rended[i + 1] = 0;
	}
}

void			ready_cellshading(t_render_opts *opts, t_cfilter f)
{
	int			i;
	t_mclr		c;

	i = -1;
	while (++i < opts->width * opts->height)
	{
		destr(opts->pixels[i], &c.ce);
		if (i % (int)opts->width >= opts->width - 5 || i < opts->width * 5 || \
			i % (int)opts->width <= 5 || \
			i / (int)opts->width >= (int)opts->height - 5)
			opts->rended[i] = restr(0, 0, 0);
		else
			apply_cellshading(&c, opts, i, f);
	}
}
