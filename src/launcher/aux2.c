/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   aux2.c                                             :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: alerandy <alerandy@student.42.fr>          +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/06/06 13:06:23 by alerandy          #+#    #+#             */
/*   Updated: 2018/06/11 13:24:27 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "interface.h"

static void	destroy_sdl(t_launch *l, int *i)
{
	if (l->prm[*i]->sdl)
	{
		SDL_DestroyTexture(l->prm[*i]->sdl->texture);
		SDL_DestroyRenderer(l->prm[*i]->sdl->renderer);
		SDL_DestroyWindow(l->prm[*i]->sdl->window);
		l->prm[*i]->sdl->window = NULL;
	}
	destroy_thrprm(l->prm[*i]);
	l->prm[*i++]->quited = 1;
}

static void	trigger_fullscreen(t_launch *l, int i, int flag, int t)
{
	SDL_SetWindowFullscreen(l->prm[i]->sdl->window, flag);
	l->prm[i]->sdl->is_fs = t;
}

void		getndestroy_rt(t_launch *l)
{
	int		i;
	int		j;

	i = 0;
	j = 0;
	l->event.type == SDL_QUIT ? l->state = QUIT : 0;
	while (i < MAXTHREAD)
		if (!l->thr[i])
			i++;
		else if (l->prm[i] && l->prm[i]->sdl && \
			((l->prm[i]->sdl && l->prm[i]->sdl->quit) || l->prm[i]->quited))
			destroy_sdl(l, &i);
		else if (l->prm[i] && l->prm[i]->sdl && ((l->prm[i]->sdl && \
			l->prm[i]->sdl->quit) || (l->prm[i]->sdl->fullscreen && \
			!l->prm[i]->sdl->is_fs)))
			trigger_fullscreen(l, i, SDL_WINDOW_FULLSCREEN_DESKTOP, 1);
		else if (l->prm[i] && l->prm[i]->sdl && ((l->prm[i]->sdl && \
			l->prm[i]->sdl->quit) || (!l->prm[i]->sdl->fullscreen && \
			l->prm[i]->sdl->is_fs)))
			trigger_fullscreen(l, i, 0, 0);
		else
			i++;
}

void		load_sdl(t_launch *launcher, t_ttf *load, t_texture **txtr)
{
	SDL_Rect				render_quad;
	static t_txt_renderer	t = (t_txt_renderer){400, 400, NULL, 0, NULL};

	render_quad = (SDL_Rect){350, 350, 51, 51};
	t.angle += 45;
	SDL_Delay(150);
	SDL_RenderFillRect(launcher->render, &(launcher->img));
	SDL_RenderCopy(launcher->render, load->texture, NULL, &(load->dstrect));
	SDL_RenderCopyEx(launcher->render, txtr[0]->texture, NULL, \
			&(render_quad), t.angle, NULL, SDL_FLIP_NONE);
	SDL_RenderPresent(launcher->render);
}
