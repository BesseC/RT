/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   l_cyli.c                                           :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: alerandy <alerandy@student.42.fr>          +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/05/21 19:50:24 by alerandy          #+#    #+#             */
/*   Updated: 2018/06/09 11:56:45 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "l_obj.h"

static void	radius(t_sdl *sdl, t_cylinder *obj)
{
	if (sdl->event->key.keysym.sym == SDLK_KP_PLUS && obj->radius < 10)
		obj->radius += 0.1;
	else if (sdl->event->key.keysym.sym == SDLK_KP_MINUS && obj->radius > 0.2)
		obj->radius -= 0.1;
}

static void	cyl_axi(t_sdl *sdl, t_cylinder *obj)
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

void		l_cyli(t_sdl *sdl, t_cylinder *obj, t_obj *parent)
{
	int		key;

	key = sdl->event->key.keysym.sym;
	radius(sdl, obj);
	if (key == SDLK_KP_4 || key == SDLK_KP_6 || key == SDLK_KP_7 || \
			key == SDLK_KP_1 || key == SDLK_KP_9 || key == SDLK_KP_3)
		cyl_axi(sdl, obj);
	posi(sdl, parent);
	texture_mv(sdl, parent);
}
