/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   sdl.c                                              :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <scornaz@student.42.fr>            +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/04/14 17:08:44 by scornaz           #+#    #+#             */
/*   Updated: 2018/06/07 16:06:31 by acourtin         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "rtv1.h"
#include "interface.h"
#include "colors.h"

static void	determine_fullscreen(t_render_opts *opts, t_sdl *sdl)
{
	while (sdl->event->key.keysym.sym == SDLK_f && \
		sdl->event->type == SDL_KEYDOWN)
		;
	if (!(SDL_GetWindowFlags(sdl->window) & SDL_WINDOW_FULLSCREEN_DESKTOP))
		while (!(SDL_GetWindowFlags(sdl->window) & \
			SDL_WINDOW_FULLSCREEN_DESKTOP))
			sdl->fullscreen = 1;
	else
		while (SDL_GetWindowFlags(sdl->window) & \
			SDL_WINDOW_FULLSCREEN_DESKTOP)
			sdl->fullscreen = 0;
	if (sdl->fullscreen == 1)
		changing_res(opts, sdl, 0, 0);
	if (sdl->fullscreen == 0)
		changing_res(opts, sdl, opts->owidth, opts->oheight);
}

static void	event_loop(t_render_opts *opts, t_sdl *sdl, t_thrprm *prm)
{
	int		id;

	id = (int)SDL_GetWindowID(SDL_GetKeyboardFocus());
	SDL_StopTextInput();
	if (sdl->event->key.keysym.sym == SDLK_q && sdl->event->type == SDL_KEYDOWN)
		while (sdl->event->type == SDL_KEYDOWN)
			sdl->quit = 1;
	else if (sdl->event->type == SDL_KEYDOWN && sdl->id == id && \
			sdl->event->key.keysym.sym == SDLK_f)
		determine_fullscreen(opts, sdl);
	else if (sdl->event->type == SDL_KEYDOWN && sdl->id == id)
		check_event(opts, sdl, prm);
}

static void	events(t_sdl *sdl, t_render_opts *opts, t_thrprm *param)
{
	while (!sdl->quit)
	{
		if (opts->it > 1)
		{
			render(opts) ? 0 : (opts->it += ITSPEED);
			sdl->is_rendering = 0;
			opts->it = opts->it - ITSPEED > 0 ? opts->it - ITSPEED : 1;
		}
		else if (opts->it == 1)
		{
			render_wait(sdl, opts);
			sdl->is_rendering = 0;
			opts->it = 0;
		}
		if (sdl->is_rendering == 0)
			change_colors(opts, sdl->filter);
		sdl->is_rendering = 1;
		SDL_UpdateTexture(sdl->texture, NULL, opts->rended,
							opts->width * sizeof(int));
		SDL_RenderClear(sdl->renderer);
		SDL_RenderCopy(sdl->renderer, sdl->texture, NULL, NULL);
		SDL_RenderPresent(sdl->renderer);
		event_loop(opts, sdl, param);
	}
}

void		init_sdl(t_render_opts *opts, t_thrprm *param)
{
	t_sdl		sdl;

	sdl = (t_sdl){NULL, 0, 0, 0, 0, param->event, 0, 0, 0, NONE};
	opts->scname = param->scn;
	param->opts = opts;
	param->sdl = &sdl;
	while (!sdl.window)
		;
	sdl.renderer = SDL_CreateRenderer(sdl.window, -1,
					SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
	sdl.texture = SDL_CreateTexture(sdl.renderer,
									SDL_PIXELFORMAT_RGB888,
									SDL_TEXTUREACCESS_STATIC,
									opts->width, opts->height);
	sdl.id = SDL_GetWindowID(sdl.window);
	sdl.event = param->event;
	change_colors(opts, sdl.filter);
	sdl.is_rendering = 1;
	opts->it = 0;
	events(&sdl, opts, param);
	while (sdl.window)
		;
}
