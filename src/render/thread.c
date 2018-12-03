/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   thread.c                                           :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/06/09 16:47:56 by scornaz           #+#    #+#             */
/*   Updated: 2018/06/09 16:52:08 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "rtv1.h"
#include "rt.h"

t_vec3f			create_ray(unsigned x, unsigned y,
						t_render_opts *opts)
{
	t_vec3f	result;

	result = ((t_vec3f){
			(2 * ((x + 0.5) * opts->config->inv_width) - 1) *
				opts->config->angle *
				opts->config->aspectratio,
				(1 - 2 * ((y + 0.5) *
				opts->config->inv_height)) * opts->config->angle,
				-1});
	return (result);
}

void			*render_f(void *render_opts)
{
	t_vec3f			col;
	t_vec3f			pos;
	t_render_opts	*opts;
	t_vec3f			raydir;

	opts = ((t_thread*)render_opts)->opts;
	pos.y = ((t_thread*)render_opts)->from - 1;
	while ((int)++pos.y < ((t_thread*)render_opts)->to)
	{
		pos.x = -1;
		while (++pos.x < opts->width / opts->it)
		{
			raydir = matrix_mul(opts->matrix, \
				create_ray(pos.x * opts->it, pos.y * opts->it, opts));
			ft_raytrace(opts->scene, &col, opts->camorig, raydir);
			pos.z = -1;
			while (++pos.z < opts->it * opts->it)
				if (pos.x * opts->it + (pos.z / opts->it) < opts->width)
					draw(opts->pixels, pos.y * opts->width * opts->it + pos.x \
					* opts->it + (pos.z / opts->it) + ((int)pos.z % opts->it) \
					* opts->width, &col);
		}
	}
	pthread_exit(0);
}

int				render(t_render_opts *opts)
{
	pthread_t		*threads;
	t_thread		*args;
	int				i;

	if (!(threads = malloc(sizeof(pthread_t) * 8)))
		return (0);
	if (!(args = malloc(sizeof(t_thread) * 8)))
		return (0);
	i = -1;
	opts->scene = ft_memalloc(sizeof(t_scene));
	ft_convert(*opts->spheres, opts->scene);
	opts->matrix = matrix_new(opts->camorig, opts->camdir, opts->roll);
	while (++i < 8)
	{
		args[i] = (t_thread){(((opts->height / opts->it)) * i) / 8, \
			(opts->height / opts->it) * (i + 1) / 8, i, opts};
		pthread_create(&threads[i], NULL, render_f, &(args[i]));
	}
	i = 0;
	while (i < 8)
		pthread_join(threads[i++], NULL);
	free_scene(opts->scene);
	free(threads);
	free(args);
	return (1);
}
