/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   colors2.c                                          :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: acourtin <acourtin@student.42.fr>          +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/05/21 10:47:56 by acourtin          #+#    #+#             */
/*   Updated: 2018/06/03 16:03:08 by acourtin         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "rtv1.h"
#include "colors.h"

void			apply_filter2(t_clr *c, t_cfilter f)
{
	if (f == BLUEISH)
		*c = (t_clr){128 + (c->r - 128) * 1.3, 128 + (c->g - 128) * 1.3,
			128 + (c->b - 128) * 0.8};
	else if (f == OLDWEST)
	{
		*c = (t_clr){(c->r * .393) + (c->g * .769) + (c->b * .189),
			(c->r * .349) + (c->g * .686) + (c->b * .168),
			(c->r * .272) + (c->g * .534) + (c->b * .131)};
		*c = (t_clr){128 + (c->r - 128) * .8,
			128 + (c->g - 128) * .7,
			128 + (c->b - 128) * .7};
	}
}

void			destr(const int cl, t_clr *c)
{
	*c = (t_clr){((int)(cl / 256) / 256),
		((int)(cl / 256)) & 0x0000FF, cl & 0x0000FF};
}

float			determine_luma(t_clr *c)
{
	return (0.299 * c->r + 0.587 * c->g + 0.114 * c->b);
}

void			destr2(const int cl, t_clr *c, int *ok, float *luma)
{
	*c = (t_clr){((int)(cl / 256) / 256),
		((int)(cl / 256)) & 0x0000FF, cl & 0x0000FF};
	*ok = 1;
	*luma = determine_luma(c);
}

int				restr(int r, int g, int b)
{
	r = r > 255 ? 255 : r;
	g = g > 255 ? 255 : g;
	b = b > 255 ? 255 : b;
	r = r < 0 ? 0 : r;
	g = g < 0 ? 0 : g;
	b = b < 0 ? 0 : b;
	return ((r * 256 * 256) + (g * 256) + b);
}
