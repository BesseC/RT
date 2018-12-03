/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   sdl2.c                                             :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: acourtin <acourtin@student.42.fr>          +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/06/06 16:33:25 by acourtin          #+#    #+#             */
/*   Updated: 2018/06/11 11:54:47 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "rtv1.h"
#include "interface.h"

static void	change_scene(t_render_opts *opts)
{
	ft_printf("objects:\n");
	while (**opts->spheres)
	{
		ft_printf("%s @ %p\n", (**opts->spheres)->tag, **opts->spheres);
		++(*opts->spheres);
	}
	ft_printf("///\n");
	++(*opts->spheres);
	if (!**opts->spheres)
	{
		while (*opts->spheres != opts->orig)
			--(*opts->spheres);
	}
}

static void	d_cam_move(t_sdl *sdl, t_render_opts *opts)
{
	if (sdl->event->key.keysym.sym == SDLK_i)
	{
		opts->camorig.x += opts->camdir.x;
		opts->camorig.y += opts->camdir.y;
		opts->camorig.z += opts->camdir.z;
	}
	else if (sdl->event->key.keysym.sym == SDLK_k)
	{
		opts->camorig.x -= opts->camdir.x;
		opts->camorig.y -= opts->camdir.y;
		opts->camorig.z -= opts->camdir.z;
	}
	else if (sdl->event->key.keysym.sym == SDLK_j)
		ft_rot_y(&opts->camdir.x, &opts->camdir.z, ASPEED);
	else if (sdl->event->key.keysym.sym == SDLK_l)
		ft_rot_y(&opts->camdir.x, &opts->camdir.z, -ASPEED);
}

static void	cam_move(t_sdl *sdl, t_render_opts *opts)
{
	if (sdl->event->key.keysym.sym == SDLK_DOWN)
		opts->camorig.z += 1;
	else if (sdl->event->key.keysym.sym == SDLK_UP)
		opts->camorig.z -= 1;
	else if (sdl->event->key.keysym.sym == SDLK_RIGHT)
		opts->camorig.x += 1;
	else if (sdl->event->key.keysym.sym == SDLK_LEFT)
		opts->camorig.x -= 1;
	else if (sdl->event->key.keysym.sym == SDLK_KP_8)
		ft_rot_x(&opts->camdir.y, &opts->camdir.z, ASPEED);
	else if (sdl->event->key.keysym.sym == SDLK_KP_5)
		ft_rot_x(&opts->camdir.y, &opts->camdir.z, -ASPEED);
	else if (sdl->event->key.keysym.sym == SDLK_KP_6)
		ft_rot_y(&opts->camdir.x, &opts->camdir.z, -ASPEED);
	else if (sdl->event->key.keysym.sym == SDLK_KP_4)
		ft_rot_y(&opts->camdir.x, &opts->camdir.z, ASPEED);
	else if (sdl->event->key.keysym.sym == SDLK_KP_9)
		ft_rot_z(&opts->camdir.x, &opts->camdir.y, -ASPEED);
	else if (sdl->event->key.keysym.sym == SDLK_KP_3)
		ft_rot_z(&opts->camdir.x, &opts->camdir.y, ASPEED);
	else if (sdl->event->key.keysym.sym == SDLK_KP_0)
		ft_rot_z(&opts->roll.x, &opts->roll.y, ASPEED);
	else if (sdl->event->key.keysym.sym == SDLK_KP_PERIOD)
		ft_rot_z(&opts->roll.x, &opts->roll.y, -ASPEED);
	d_cam_move(sdl, opts);
}

static void	key_event(t_render_opts *opts, t_sdl *sdl)
{
	cam_move(sdl, opts);
	if (sdl->event->key.keysym.sym == SDLK_a)
		change_scene(opts);
	else if (sdl->event->key.keysym.sym == SDLK_s)
		snap_screen(opts, sdl);
	else if (sdl->event->key.keysym.sym == SDLK_n)
		new_scene(opts);
	sdl->event->key.keysym.sym == 27 ? sdl->quit = 1 : 0;
	while ((sdl->event->key.keysym.sym == 27 || sdl->event->key.keysym.sym == \
		SDLK_a || sdl->event->key.keysym.sym == SDLK_s || \
		sdl->event->key.keysym.sym == SDLK_n) \
		&& sdl->event->type == SDL_KEYDOWN)
		;
}

void		check_event(t_render_opts *opts, t_sdl *sdl, t_thrprm *prm)
{
	if (sdl->event->key.keysym.sym != SDLK_s)
		opts->it = ITRES;
	if (prm->sobj)
		obj_key(opts, sdl, prm->sobj);
	else
		key_event(opts, sdl);
}
