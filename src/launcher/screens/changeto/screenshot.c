/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   screenshot.c                                       :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: acourtin <acourtin@student.42.fr>          +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/06/05 11:49:06 by acourtin          #+#    #+#             */
/*   Updated: 2018/06/08 20:58:43 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "rtv1.h"
#include "interface.h"

static void		remplace_space(char *str)
{
	int		i;

	i = -1;
	while (str[++i])
	{
		if (str[i] == ' ' || str[i] == '?' || str[i] == '\n' || str[i] == ':' \
				|| str[i] == '/')
			str[i] = '_';
	}
}

static char		*get_filename(t_render_opts *opts, struct tm *time)
{
	char		*filename;
	char		*filename2;
	char		*timeinfo;

	timeinfo = ft_strdup(asctime(time));
	filename = ft_strjoin("screenshots/", opts->scname);
	filename2 = ft_strjoin(filename, "_");
	ft_strdel(&filename);
	filename = ft_strjoin(filename2, timeinfo);
	ft_strdel(&filename2);
	filename2 = ft_strjoin(filename, ".bmp");
	remplace_space(filename2 + 13);
	ft_strdel(&filename);
	ft_strdel(&timeinfo);
	return (filename2);
}

void			snap_screen(t_render_opts *opts, t_sdl *sdl)
{
	SDL_Surface			*screenshot;
	time_t				rawtime;
	struct tm			*timeinfo;
	char				*filename;

	screenshot = SDL_CreateRGBSurface(0, opts->width, opts->height, \
			32, 0, 0, 0, 0);
	SDL_RenderReadPixels(sdl->renderer, NULL, SDL_PIXELFORMAT_RGB888, \
		screenshot->pixels, screenshot->pitch);
	if (screenshot)
	{
		time(&rawtime);
		timeinfo = localtime(&rawtime);
		filename = get_filename(opts, timeinfo);
		if (SDL_SaveBMP(screenshot, filename) == 0)
			ft_printf("*snap* %s\n", filename);
		else
			ft_putendl("Failed to screenshot.\nDiskVolume may be full.\n");
		SDL_FreeSurface(screenshot);
		ft_strdel(&filename);
	}
}
