/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   l_fcyl.c                                           :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: alerandy <alerandy@student.42.fr>          +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/05/21 19:48:31 by alerandy          #+#    #+#             */
/*   Updated: 2018/06/08 13:15:26 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "l_obj.h"

static void	con_axi(t_sdl *sdl, t_fcylinder *obj)
{
	if (sdl->event->key.keysym.sym == SDLK_KP_4)
		ft_rot_y(&obj->dir.x, &obj->dir.z, -ASPEED);
	else if (sdl->event->key.keysym.sym == SDLK_KP_6)
		ft_rot_y(&obj->dir.x, &obj->dir.z, ASPEED);
	else if (sdl->event->key.keysym.sym == SDLK_KP_7)
		ft_rot_x(&obj->dir.y, &obj->dir.z, -ASPEED);
	else if (sdl->event->key.keysym.sym == SDLK_KP_1)
		ft_rot_x(&obj->dir.y, &obj->dir.z, ASPEED);
	else if (sdl->event->key.keysym.sym == SDLK_KP_9)
		ft_rot_z(&obj->dir.x, &obj->dir.y, -ASPEED);
	else if (sdl->event->key.keysym.sym == SDLK_KP_3)
		ft_rot_z(&obj->dir.x, &obj->dir.y, ASPEED);
}

static void	posi(t_sdl *sdl, t_fcylinder *obj)
{
	if (sdl->event->key.keysym.sym == SDLK_DOWN)
		obj->base.y -= 0.2;
	else if (sdl->event->key.keysym.sym == SDLK_UP)
		obj->base.y += 0.2;
	else if (sdl->event->key.keysym.sym == SDLK_RIGHT)
		obj->base.x += 0.2;
	else if (sdl->event->key.keysym.sym == SDLK_LEFT)
		obj->base.x -= 0.2;
	else if (sdl->event->key.keysym.sym == SDLK_KP_5)
		obj->base.z -= 0.2;
	else if (sdl->event->key.keysym.sym == SDLK_KP_8)
		obj->base.z += 0.2;
}

static void	radius(t_sdl *sdl, t_fcylinder *obj)
{
	if (sdl->event->key.keysym.sym == SDLK_KP_PLUS && obj->radius < 5)
		obj->radius += 0.1;
	else if (sdl->event->key.keysym.sym == SDLK_KP_MINUS && obj->radius > 0.3)
		obj->radius -= 0.1;
}

static void	height(t_sdl *sdl, t_fcylinder *obj)
{
	if (sdl->event->key.keysym.sym == SDLK_KP_DIVIDE && obj->size > 0.4)
		obj->size -= 0.2;
	else if (sdl->event->key.keysym.sym == SDLK_KP_MULTIPLY && obj->size < 5)
		obj->size += 0.2;
}

void		l_fcyl(t_sdl *sdl, t_fcylinder *obj, t_obj *parent)
{
	int		key;

	key = sdl->event->key.keysym.sym;
	(void)parent;
	if (key == SDLK_KP_4 || key == SDLK_KP_6 || key == SDLK_KP_7 || \
			key == SDLK_KP_1 || key == SDLK_KP_9 || key == SDLK_KP_3)
		con_axi(sdl, obj);
	posi(sdl, obj);
	height(sdl, obj);
	radius(sdl, obj);
}
