/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   l_sphe.c                                           :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: alerandy <alerandy@student.42.fr>          +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/05/21 19:52:43 by alerandy          #+#    #+#             */
/*   Updated: 2018/06/09 11:57:02 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "l_obj.h"

static void	radius(t_sdl *sdl, t_sphere *obj)
{
	if (sdl->event->key.keysym.sym == SDLK_KP_PLUS && obj->radius < 10)
		obj->radius += 0.1;
	else if (sdl->event->key.keysym.sym == SDLK_KP_MINUS && obj->radius > 0.4)
		obj->radius -= 0.1;
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

static void	texture_mv(t_sdl *sdl, t_obj *obj)
{
	if (sdl->event->key.keysym.sym == SDLK_i)
		obj->txtr_pos.y -= 0.1;
	else if (sdl->event->key.keysym.sym == SDLK_k)
		obj->txtr_pos.y += 0.1;
	else if (sdl->event->key.keysym.sym == SDLK_j)
		obj->txtr_pos.x += 0.1;
	else if (sdl->event->key.keysym.sym == SDLK_l)
		obj->txtr_pos.x -= 0.1;
	else if (sdl->event->key.keysym.sym == SDLK_u)
		obj->txtr_ang.x += 1;
	else if (sdl->event->key.keysym.sym == SDLK_m)
		obj->txtr_ang.x -= 1;
}

void		l_sphe(t_sdl *sdl, t_sphere *obj, t_obj *parent)
{
	radius(sdl, obj);
	posi(sdl, parent);
	texture_mv(sdl, parent);
}
