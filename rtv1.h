/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   rtv1.h                                             :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/06/09 16:51:15 by scornaz           #+#    #+#             */
/*   Updated: 2018/06/10 14:04:37 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef RTV1_H
# define RTV1_H

# include <SDL.h>
# include <pthread.h>
# include "libft.h"
# include "printf.h"
# include "vec3f.h"
# include "object.h"
# include "parser.h"
# include "globals.h"
# include "rt.h"
# include "colors.h"

# define BACKGROUND 0.1, 0.1, 0.1
# define BIAS		1e-12
# define PHONG  8
# define MAX_RAY_DEPTH	5
# define FX			0
# define DEBUG		0
# define NO_SHADOW 0
# define SPEC 1
# define SMOOTH_LIGHT 0
# define ITRES 10
# define ITSPEED 3

typedef struct		s_config{
	double			inv_width;
	double			inv_height;
	double			fov;
	double			aspectratio;
	double			angle;
}					t_config;

typedef struct		s_render_opts{
	t_obj			***spheres;
	int				*pixels;
	int				*rended;
	t_config		*config;
	t_vec3f			camorig;
	t_vec3f			camdir;
	double			width;
	double			height;
	double			owidth;
	double			oheight;
	t_33mat			matrix;
	t_obj			**orig;
	t_vec3f			roll;
	int				it;
	t_scene			*scene;
	char			*scname;
}					t_render_opts;

typedef struct		s_sdl{
	SDL_Window		*window;
	SDL_Renderer	*renderer;
	SDL_Texture		*texture;
	int				quit;
	int				id;
	SDL_Event		*event;
	int				is_rendering;
	int				fullscreen;
	int				is_fs;
	t_cfilter		filter;
}					t_sdl;

typedef struct		s_thread{
	int				from;
	int				to;
	int				i;
	t_render_opts	*opts;
}					t_thread;

t_obj				*trace(t_ray *hit, t_obj **spheres, int depth, \
		t_vec3f *color);
void				diffuse(t_obj **spheres, t_obj *sphere, t_ray *hit);
void				effects(t_obj **spheres, t_obj *sphere, t_ray *hit, \
		int depth);
int					render(t_render_opts *opts);
void				draw(int *pixel, int index, t_vec3f *colors);
t_vec3f				create_ray(unsigned x, unsigned y, t_render_opts *opts);
void				change_colors(t_render_opts *opts, t_cfilter f);

void				ready_cellshading(t_render_opts *opts, t_cfilter f);
void				get_lumas(t_mclr *c, t_render_opts *opts, int i);
void				get_lumas2(t_mclr *c, t_render_opts *opts, int i);
void				render_wait(t_sdl *sdl, t_render_opts *opts);
void				changing_res(t_render_opts *opts, t_sdl *sdl, int w, int h);

int					new_scene(t_render_opts *opts);
#endif
