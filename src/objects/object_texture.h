/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   object_texture.h                                   :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/06/09 12:52:45 by scornaz           #+#    #+#             */
/*   Updated: 2018/06/09 18:00:26 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef OBJECT_TEXTURE_H
# define OBJECT_TEXTURE_H

# include "vec3f.h"
# include <SDL.h>

typedef	enum	e_text_type{
	PATTERN, ASSET, TXT_NONE
}				t_text_type;

typedef struct	s_obj_texture{
	char			*name;
	t_text_type		type;
	SDL_Surface		*surface;
	unsigned		pattern;
}				t_obj_texture;

#endif
