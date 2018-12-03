/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   multi_ttf.c                                        :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: acourtin <acourtin@student.42.fr>          +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/05/03 16:26:27 by acourtin          #+#    #+#             */
/*   Updated: 2018/05/26 15:48:01 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include <stdarg.h>
#include "interface.h"

t_ttf			**multi_ttf(int nb, SDL_Renderer *renderer, char *font, \
		t_pos pos, ...)
{
	va_list			ap;
	t_ttf			**multi_ttf;
	int				i;

	multi_ttf = ft_memalloc(sizeof(t_ttf*) * (nb + 1));
	va_start(ap, pos);
	i = 0;
	while (i < nb)
	{
		multi_ttf[i] = ttf_new(renderer, va_arg(ap, char*), font, pos);
		i > 0 ? multi_ttf[i]->dstrect.y = multi_ttf[i - 1]->texh * (i) + \
										multi_ttf[i]->texh / 10 + pos.y : 0;
		i++;
	}
	va_end(ap);
	return (multi_ttf);
}

void			m_ttf_destroy(t_ttf **text)
{
	int			i;

	i = 0;
	if (text)
	{
		while (text[i])
		{
			ttf_destroy(text[i]);
			text[i] = NULL;
			i++;
		}
		free(text);
	}
	text = NULL;
}
