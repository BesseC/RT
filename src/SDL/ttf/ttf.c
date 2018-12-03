/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ttf.c                                              :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: acourtin <acourtin@student.42.fr>          +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/04/05 12:29:11 by acourtin          #+#    #+#             */
/*   Updated: 2018/05/26 15:19:44 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "ttf.h"
#include "sdl_mouse.h"

t_ttf		*ttf_new(SDL_Renderer *renderer, char *str, char *font, t_pos pos)
{
	t_ttf	*t;

	t = NULL;
	if (!(t = ft_memalloc(sizeof(t_ttf))))
		return (NULL);
	t->texw = 0;
	t->texh = 0;
	t->font = TTF_OpenFont(font, pos.z);
	t->color.r = 125;
	t->color.g = 125;
	t->color.b = 125;
	t->renderer = renderer;
	t->surface = TTF_RenderText_Solid(t->font, str, t->color);
	t->texture = SDL_CreateTextureFromSurface(t->renderer, t->surface);
	SDL_QueryTexture(t->texture, NULL, NULL, &t->texw, &t->texh);
	t->dstrect.x = pos.x;
	t->dstrect.y = pos.y;
	t->dstrect.w = t->texw;
	t->dstrect.h = t->texh;
	return (t);
}

void		ttf_destroy(t_ttf *t)
{
	if (t)
	{
		if (t->texture)
			SDL_DestroyTexture(t->texture);
		if (t->surface)
			SDL_FreeSurface(t->surface);
		if (t->font)
			TTF_CloseFont(t->font);
		t->font = NULL;
		free(t);
	}
	t = NULL;
}

t_ttf		*ttf_newb(SDL_Renderer *renderer, char *str, t_button *button, \
		char *font)
{
	t_ttf	*t;
	t_pos	pos;
	int		i;

	t = NULL;
	pos.x = 0;
	pos.y = 0;
	pos.z = 27;
	i = ft_strlen(str);
	while (i > 0 && str[i] != '/')
		i--;
	str[i] == '/' ? i++ : 0;
	if (!(t = ttf_new(renderer, str + i, font, pos)))
		return (NULL);
	t->dstrect.x = button->position.x + (button->width - t->texw) / 2;
	ft_strlen(str) >= 9 ? t->dstrect.x -= 7 : 0;
	t->dstrect.y = button->position.y + (button->height - t->texh) / 2;
	t->tmpy = t->dstrect.y;
	t->tmpx = t->dstrect.x;
	return (t);
}
