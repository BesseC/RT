/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   l_plan.c                                           :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: alerandy <alerandy@student.42.fr>          +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/05/21 19:00:33 by alerandy          #+#    #+#             */
/*   Updated: 2018/06/08 10:54:45 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "l_obj.h"

static void	pla_axi(t_sdl *sdl, t_plane *obj)
{
	if (sdl->event->key.keysym.sym == SDLK_KP_4)
		ft_rot_y(&obj->axis.x, &obj->axis.z, -ASPEED);
	else if (sdl->event->key.keysym.sym == SDLK_KP_6)
		ft_rot_y(&obj->axis.x, &obj->axis.z, ASPEED);
	else if (sdl->event->key.keysym.sym == SDLK_KP_7)
		ft_rot_x(&obj->axis.y, &obj->axis.z, -ASPEED);
	else if (sdl->event->key.keysym.sym == SDLK_KP_1)
		ft_rot_x(&obj->axis.y, &obj->axis.z, ASPEED);
	else if (sdl->event->key.keysym.sym == SDLK_KP_9)
		ft_rot_z(&obj->axis.x, &obj->axis.y, -ASPEED);
	else if (sdl->event->key.keysym.sym == SDLK_KP_3)
		ft_rot_z(&obj->axis.x, &obj->axis.y, ASPEED);
}

static void	posi(t_sdl *sdl, t_obj *obj)
{
	if (sdl->event->key.keysym.sym == SDLK_DOWN)
		obj->position.y -= 0.2;
	else if (sdl->event->key.keysym.sym == SDLK_UP)
		obj->position.y += 0.2;
	else if (sdl->event->key.keysym.sym == SDLK_RIGHT)
		obj->position.x += 0.2;
	else if (sdl->event->key.keysym.sym == SDLK_LEFT)
		obj->position.x -= 0.2;
	else if (sdl->event->key.keysym.sym == SDLK_KP_5)
		obj->position.z -= 0.2;
	else if (sdl->event->key.keysym.sym == SDLK_KP_8)
		obj->position.z += 0.2;
}

static void	d_size(t_sdl *sdl, t_plane *obj)
{
	if (sdl->event->key.keysym.sym == SDLK_KP_PLUS)
		obj->size += 0.2;
	else if (sdl->event->key.keysym.sym == SDLK_KP_MINUS && obj->size > 0.2)
		obj->size -= 0.2;
}

void		l_plan(t_sdl *sdl, t_plane *obj, t_obj *parent)
{
	int		key;

	key = sdl->event->key.keysym.sym;
	if (key == SDLK_KP_4 || key == SDLK_KP_6 || key == SDLK_KP_7 || \
			key == SDLK_KP_1 || key == SDLK_KP_9 || key == SDLK_KP_3)
		pla_axi(sdl, obj);
	posi(sdl, parent);
	if (ft_strequ(parent->tag, "disque"))
		d_size(sdl, obj);
}
