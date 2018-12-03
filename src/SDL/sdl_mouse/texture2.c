/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   texture2.c                                         :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: acourtin <marvin@42.fr>                    +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/03/28 17:44:28 by acourtin          #+#    #+#             */
/*   Updated: 2018/03/28 17:45:27 by acourtin         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "sdl_mouse.h"

void			texture_set_color(t_texture *self, int r, int g, int b)
{
	SDL_SetTextureColorMod(self->texture, r, g, b);
}

void			texture_set_blend_mode(t_texture *self, SDL_BlendMode blending)
{
	SDL_SetTextureBlendMode(self->texture, blending);
}

void			texture_set_alpha(t_texture *self, int alpha)
{
	SDL_SetTextureAlphaMod(self->texture, alpha);
}
