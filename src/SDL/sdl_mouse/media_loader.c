/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   media_loader.c                                     :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: acourtin <marvin@42.fr>                    +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/04/11 14:18:06 by acourtin          #+#    #+#             */
/*   Updated: 2018/05/25 14:14:05 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include <stdarg.h>
#include "sdl_mouse.h"
#include "libft.h"

/*
**Une fonction pour charger automatiquement des images reactives
**
** args: nb
** les functions a arguments variables en C doivent indiquer
** le nb d'argument en debut de fonction
**
** args: ref vers texture
** le chemin ou trouver le fichier des images. On doit indiquer
** une 'spritesheet' c'est a dire une image avec plusieurs images
** pour pouvoir simuler differents 'etats' (l'image change selon si
** la souris est dessus ou non, si elle clique etc.)
**
** args: nb_states et clips
** il faut donc indiquer le nb d'etats possibles, et la taille veritable
** de l'image (si l'image fait en vrai 100*100, la spritesheet fera 100*
**		nb_states_possibles * 100)
**
** le reste des arguments indique le nb de buttons similaires a creer,
**		et leur position
** x y
*/

t_button	**buttons_loader(int nb, t_texture *texture,
		int nb_states, SDL_Rect size, ...)
{
	va_list		ap;
	t_button	**ptr;
	int			i;
	int			j;

	i = -1;
	ptr = (t_button**)ft_memalloc(sizeof(t_button*) * (nb + 1));
	va_start(ap, size);
	while (++i < nb)
	{
		ptr[i] = button_new(va_arg(ap, int), va_arg(ap, int), \
				va_arg(ap, int), va_arg(ap, int));
		ptr[i]->texture = texture;
		j = -1;
		while (++j < nb_states)
		{
			ptr[i]->clips[j] = size;
			ptr[i]->clips[j].y = size.y * j;
		}
	}
	ptr[i] = NULL;
	va_end(ap);
	return (ptr);
}

t_texture	**textures_loader(int nb, SDL_Renderer *renderer, ...)
{
	va_list		ap;
	t_texture	**ptr;
	int			i;

	i = -1;
	ptr = (t_texture**)malloc(sizeof(t_texture*) * (nb + 1));
	va_start(ap, renderer);
	while (++i < nb)
	{
		ptr[i] = texture_new();
		if (!(ptr[i]->
				texture_load_from_file(ptr[i], va_arg(ap, char*), renderer)))
			return (0);
	}
	ptr[i] = 0;
	va_end(ap);
	return (ptr);
}
