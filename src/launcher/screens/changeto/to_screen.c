/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   to_screen.c                                        :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: alerandy <alerandy@student.42.fr>          +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/05/04 05:37:09 by alerandy          #+#    #+#             */
/*   Updated: 2018/06/04 18:37:03 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "interface.h"

void		*to_newscreen(void *param)
{
	t_launch	*launcher;

	launcher = param;
	refresh_ls(launcher);
	launcher->state = NEW;
	return (NULL);
}

void		*to_mainscreen(void *param)
{
	t_launch	*launcher;

	launcher = param;
	launcher->state = MSCREEN;
	return (NULL);
}

void		*to_rtscreen(void *param)
{
	t_launch	*launcher;

	launcher = param;
	launcher->state = RTS;
	return (NULL);
}

void		*to_rtopt(void *param)
{
	t_rt	*opts;

	opts = param;
	opts->launcher->state = OPTS;
	opts->thr->sdl ? SDL_RaiseWindow(opts->thr->sdl->window) : 0;
	rt_opts(opts);
	return (NULL);
}

void		*open_scn(void *param)
{
	t_thrprm	*event;

	event = param;
	through_argv(event);
	event->quited = 1;
	while (1)
		SDL_Delay(100000000);
	pthread_exit(NULL);
}
