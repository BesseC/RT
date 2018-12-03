/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   l_fcon.c                                           :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: alerandy <alerandy@student.42.fr>          +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/05/21 19:48:31 by alerandy          #+#    #+#             */
/*   Updated: 2018/06/11 11:57:26 by cbesse           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "l_obj.h"

static void	angle(t_sdl *sdl, t_fcone2 *obj)
{
	if (sdl->event->key.keysym.sym == SDLK_KP_MINUS && obj->angle > 1)
		obj->angle -= 1;
	else if (sdl->event->key.keysym.sym == SDLK_KP_PLUS && obj->angle < 180)
		obj->angle += 1;
}

static void	con_axi(t_sdl *sdl, t_fcone2 *obj)
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

static void	posi(t_sdl *sdl, t_fcone2 *obj)
{
	if (sdl->event->key.keysym.sym == SDLK_DOWN)
		obj->tip_position.y -= 0.2;
	else if (sdl->event->key.keysym.sym == SDLK_UP)
		obj->tip_position.y += 0.2;
	else if (sdl->event->key.keysym.sym == SDLK_RIGHT)
		obj->tip_position.x += 0.2;
	else if (sdl->event->key.keysym.sym == SDLK_LEFT)
		obj->tip_position.x -= 0.2;
	else if (sdl->event->key.keysym.sym == SDLK_KP_5)
		obj->tip_position.z -= 0.2;
	else if (sdl->event->key.keysym.sym == SDLK_KP_8)
		obj->tip_position.z += 0.2;
}

static void	height(t_sdl *sdl, t_fcone2 *obj)
{
	if (sdl->event->key.keysym.sym == SDLK_KP_DIVIDE && obj->size > 0.4)
	{
		obj->size -= 0.2;
		obj->mid -= 0.1;
	}
	else if (sdl->event->key.keysym.sym == SDLK_KP_MULTIPLY && obj->size < 5)
	{
		obj->size += 0.2;
		obj->mid += 0.1;
	}
}

void		l_fcon(t_sdl *sdl, t_fcone2 *obj, t_obj *parent)
{
	int		key;

	key = sdl->event->key.keysym.sym;
	(void)parent;
	angle(sdl, obj);
	if (key == SDLK_KP_4 || key == SDLK_KP_6 || key == SDLK_KP_7 || \
			key == SDLK_KP_1 || key == SDLK_KP_9 || key == SDLK_KP_3)
		con_axi(sdl, obj);
	posi(sdl, obj);
	height(sdl, obj);
}
