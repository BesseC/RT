/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   texture.c                                          :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: acourtin <marvin@42.fr>                    +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/03/28 16:16:39 by acourtin          #+#    #+#             */
/*   Updated: 2018/05/22 11:44:42 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "sdl_mouse.h"

int				texture_load_from_file(t_texture *self, const char *path, \
		SDL_Renderer *renderer)
{
	SDL_Texture	*new_texture;
	SDL_Surface	*loaded_surface;

	loaded_surface = NULL;
	new_texture = NULL;
	loaded_surface = IMG_Load(path);
	if (loaded_surface)
	{
		SDL_SetColorKey(loaded_surface, SDL_TRUE, \
			SDL_MapRGB(loaded_surface->format, 0, 0xff, 0xFF));
		new_texture = SDL_CreateTextureFromSurface(renderer, loaded_surface);
		if (new_texture)
		{
			self->width = loaded_surface->w;
			self->height = loaded_surface->h;
		}
		SDL_FreeSurface(loaded_surface);
	}
	self->texture = new_texture;
	return (new_texture != NULL);
}

void			texture_free(t_texture *self)
{
	if (self->texture)
	{
		SDL_DestroyTexture(self->texture);
		self->texture = NULL;
	}
	free(self);
}

void			texture_render(t_texture *self, t_txt_renderer *t, \
		SDL_Renderer *renderer)
{
	SDL_Rect render_quad;

	render_quad = (SDL_Rect){t->x, t->y, self->width, self->height};
	if (t->clip != NULL)
	{
		render_quad.w = t->clip->w;
		render_quad.h = t->clip->h;
	}
	SDL_RenderCopy(renderer, self->texture, t->clip, &render_quad);
}

t_texture		*texture_new(void)
{
	t_texture	*texture;

	texture = (t_texture*)malloc(sizeof(*texture));
	texture->width = 0;
	texture->height = 0;
	texture->texture = 0;
	texture->texture_free = texture_free;
	texture->texture_load_from_file = texture_load_from_file;
	texture->texture_set_color = texture_set_color;
	texture->texture_set_blend_mode = texture_set_blend_mode;
	texture->texture_set_alpha = texture_set_alpha;
	texture->texture_render = texture_render;
	return (texture);
}
