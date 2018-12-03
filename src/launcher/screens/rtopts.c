/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   rtopts.c                                           :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: alerandy <alerandy@student.42.fr>          +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/05/09 14:05:11 by alerandy          #+#    #+#             */
/*   Updated: 2018/06/09 19:22:17 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "interface.h"
#include "rtv1.h"

static void		setoptsbtn(t_launch *launcher, t_button **btns, \
		t_thrprm *param, t_texture **txtr)
{
	int		i;
	int		j;
	char	*filter[12];

	filters_names(filter);
	i = -1;
	btns[0] = button_new(15, 125, 175, 36);
	while (++i < 12)
	{
		j = i;
		if (j > 7)
			j++;
		i == 0 ? 0 : (btns[i] = button_new(20 + (j / 8) * 200, 175 \
			+ (50 * (j % 8)), 175, 36));
		btns[i]->func = !i ? (void*)&to_rtscreen : (void *)&i;
		btns[i]->param = !i ? (void*)launcher : (void *)param;
		btns[i]->texture = !i ? txtr[1] : txtr[0];
		btns[i]->id = !i ? 0 : i;
		btns[i]->t = ttf_newb(launcher->render, filter[i], \
				btns[i], "assets/28 Days Later.ttf");
	}
	i = -1;
	while (++i < 12 * 4)
		btns[i / 4]->clips[i % 4] = (SDL_Rect){0, 36 * (i % 4), 175, 36};
}

static void		slctd(t_launch *launcher, t_rt *opts)
{
	t_ttf		**info;
	t_ttf		**value;
	char		*tmp;

	if (opts->launcher->state != OPTS || !opts->thr->sdl || !opts->selected)
		return ;
	info = multi_ttf(2, launcher->render, "assets/bebas.ttf",
		(t_pos){500, 240, 30}, opts->selected->tag, "Position :");
	value = ft_memalloc(sizeof(t_ttf *) * (6 + 1));
	tmp = ft_itoa(opts->selected->position.x);
	tmp ? value[0] = ttf_new(launcher->render, tmp, "assets/bebas.ttf", \
		(t_pos){600, 289, 20}) : 0;
	tmp ? ft_strdel(&tmp) : 0;
	tmp = ft_itoa(opts->selected->position.y);
	tmp ? value[1] = ttf_new(launcher->render, tmp, "assets/bebas.ttf", \
		(t_pos){value[0]->texw + value[0]->dstrect.x + 10, 289, 20}) : 0;
	tmp ? ft_strdel(&tmp) : 0;
	tmp = ft_itoa(opts->selected->position.z);
	tmp ? value[2] = ttf_new(launcher->render, tmp, "assets/bebas.ttf", \
		(t_pos){value[1]->texw + value[1]->dstrect.x + 10, 289, 20}) : 0;
	tmp ? ft_strdel(&tmp) : 0;
	tab_render(launcher, value);
	tab_render(launcher, info);
	m_ttf_destroy(value);
	m_ttf_destroy(info);
}

static void		loading(t_launch *launcher, t_rt *opts)
{
	t_texture		**txtr;
	t_ttf			*load;
	t_txt_renderer	t;
	SDL_Rect		render_quad;

	if (!(txtr = textures_loader(1, launcher->render, "assets/loading.png")))
		usage(40);
	load = ttf_new(launcher->render, "Please Wait...", \
			"assets/docteur_atomic.ttf", (t_pos){0, 150, 200});
	t = (t_txt_renderer){400, 400, NULL, 0, NULL};
	render_quad = (SDL_Rect){350, 350, 51, 51};
	while (opts->thr->sdl && opts->thr->opts->it > 0)
	{
		t.angle += 45;
		SDL_Delay(150);
		SDL_RenderFillRect(launcher->render, &(launcher->img));
		SDL_RenderCopy(launcher->render, load->texture, NULL, &(load->dstrect));
		SDL_RenderCopyEx(launcher->render, txtr[0]->texture, NULL, \
				&(render_quad), t.angle, NULL, SDL_FLIP_NONE);
		SDL_RenderPresent(launcher->render);
	}
	texture_free(txtr[0]);
	free(txtr);
	ttf_destroy(load);
	opts->selected && opts->thr->sdl ? opts->thr->opts->it = ITRES : 0;
}

static void		run(t_launch *launcher, t_rt *opts, t_ttf **title, \
		t_button **btns)
{
	int			i;

	while (opts->launcher->state == OPTS)
	{
		SDL_WaitEvent(&(launcher->event));
		getndestroy_rt(launcher);
		!opts->thr->sdl ? launcher->state = RTS : 0;
		SDL_RenderFillRect(launcher->render, &(launcher->img));
		tab_render(launcher, title);
		slctd(launcher, opts);
		opts->thr->sdl ? getobj(opts) : 0;
		i = 0;
		while (btns[i])
		{
			watch_btn(launcher, btns[i]);
			i == 0 && btns[i]->trigger ? btns[i]->func(btns[i]->param) : 0;
			if (btns[++i] && btns[i]->trigger)
				filter_chng(btns[i]->param, btns[i]);
		}
		SDL_RenderPresent(launcher->render);
	}
	opts->selected ? opts->selected->surface_color.x -= 255 : 0;
	loading(launcher, opts);
	opts->thr->sdl ? opts->selected = 0 : 0;
	opts->thr->sdl ? opts->thr->sobj = 0 : 0;
}

void			rt_opts(t_rt *opts)
{
	t_ttf		**title;
	t_launch	*launcher;
	t_button	**btns;
	t_texture	**txtr;

	btns = ft_memalloc(sizeof(t_button *) * (12 + 1));
	title = ft_memalloc(sizeof(t_ttf *) * (4 + 1));
	launcher = opts->launcher;
	if (!(txtr = textures_loader(2, opts->launcher->render,
					"assets/button2.png", "assets/check.png")))
		usage(3);
	title[0] = ttf_new(launcher->render, opts->thr->scn, \
			"assets/bebas.ttf", (t_pos){15, 10, 70});
	title[1] = ttf_new(launcher->render, \
			"here you change the setting(s) of the selected RT", \
			"assets/bebas.ttf", (t_pos){15, 80, 20});
	title[2] = ttf_new(launcher->render, "Filtres :", \
			"assets/bebas.ttf", (t_pos){15, 180, 35});
	title[3] = ttf_new(launcher->render, "Objet selectionne :", \
			"assets/bebas.ttf", (t_pos){490, 195, 35});
	setoptsbtn(launcher, btns, opts->thr, txtr);
	run(launcher, opts, title, btns);
	delete_mtxtr(txtr);
	btn_clean(btns);
	m_ttf_destroy(title);
}
