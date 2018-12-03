/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   main.c                                             :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <scornaz@student.42.fr>            +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/04/14 16:55:18 by scornaz           #+#    #+#             */
/*   Updated: 2018/06/11 11:56:51 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "rtv1.h"
#include "object.h"
#include "parser.h"
#include "mini_ls.h"
#include "rt.h"

void		draw(int *pixel, int index, t_vec3f *colors)
{
	int	color;

	color = 0;
	if (SMOOTH_LIGHT)
	{
		color = (int)(colors->z / (colors->z + 1) * 255);
		color += (int)(colors->y / (colors->y + 1) * 255) << 8;
		color += (int)(colors->x / (colors->x + 1) * 255) << 16;
	}
	else
	{
		color = (int)(fmin(1.0, colors->z) * 255);
		color += (int)(fmin(1.0, colors->y) * 255) << 8;
		color += (int)(fmin(1.0, colors->x) * 255) << 16;
	}
	pixel[index] = color;
}

static void	set(t_config *config, t_conf *conf, t_render_opts *opts, \
		int *screen)
{
	int		*rended;

	rended = ft_memalloc(sizeof(int) * conf->globals.width \
			* conf->globals.height);
	*config = (t_config){1 / (double)conf->globals.width, 1 /
						(double)conf->globals.height,
						60, conf->globals.width /
						(double)conf->globals.height, 0};
	(*config).angle = tan(M_PI * 0.5 * (*config).fov / 180.0);
	*opts = (t_render_opts){&conf->objects, screen, rended, &(*config), \
		conf->globals.from, conf->globals.to, conf->globals.width, \
		conf->globals.height, conf->globals.width, conf->globals.height, \
		matrix_new(conf->globals.from, conf->globals.to, (t_vec3f){0, 1, 0}), \
		conf->objects, (t_vec3f){0, 1, 0}, 1, 0, 0};
}

void		through_argv(t_thrprm *param)
{
	t_config		config;
	t_conf			*conf;
	t_render_opts	opts;
	t_obj			**tmp_obj;

	ft_bzero(&opts, sizeof(t_render_opts));
	!(conf = read_configuration(param->scn, "configs/rules")) ? \
			(param->quited = 1) : 0;
	!param->quited && param->width ? conf->globals.width = param->width : 0;
	!param->quited && param->height ? conf->globals.height = param->height : 0;
	!param->quited && !(opts.pixels = malloc(sizeof(int) * \
				conf->globals.width * conf->globals.height)) ? \
									(param->quited = 1) : 0;
	!param->quited ? set(&config, conf, &opts, opts.pixels) : 0;
	conf ? tmp_obj = conf->objects : 0;
	!param->quited ? render(&opts) : 0;
	!param->quited ? init_sdl(&opts, param) : 0;
	opts.pixels ? free(opts.pixels) : 0;
	opts.rended ? free(opts.rended) : 0;
	conf ? ft_lstdel(&conf->tmp_objects, object_del) : 0;
	conf ? free(tmp_obj) : 0;
	conf ? free(conf) : 0;
	param->sobj = 0;
}

int			main(void)
{
	ft_putendl("Coucou 🤡");
	mini_ls();
	ft_putendl("Au revoir ❤️");
	return (0);
}
