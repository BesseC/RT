/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   launcher.c                                         :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: alerandy <alerandy@student.42.fr>          +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/04/26 13:13:27 by alerandy          #+#    #+#             */
/*   Updated: 2018/05/26 15:17:53 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "interface.h"

void			watch_btn(t_launch *launcher, t_button *buttons)
{
	if (buttons)
	{
		if (1 == SDL_GetWindowID(SDL_GetMouseFocus()))
			buttons->button_handle_event(buttons, &(launcher->event));
		buttons->button_render(buttons, launcher->render);
		buttons->t ? SDL_RenderCopy(launcher->render, buttons->t->texture, \
				NULL, &(buttons->t->dstrect)) : 0;
	}
}

static void		runner(t_launch *launcher, \
		t_button **ms_btns, t_texture **txtr)
{
	launcher->state = MSCREEN;
	while (!launcher->quit)
	{
		launcher->state == NEW ? new_rt(launcher, txtr) : 0;
		launcher->state == MSCREEN ? mainscreen(launcher, ms_btns) : 0;
		launcher->state == RTS ? rtscreen(launcher, txtr) : 0;
		launcher->state == QUIT ? launcher->quit = 1 : 0;
	}
}

static void		set(t_launch *launcher)
{
	int			i;

	launcher->prm = ft_memalloc(sizeof(t_thrprm *) * MAXTHREAD);
	i = -1;
	while (++i < MAXTHREAD && launcher->prm)
		launcher->prm[i] = ft_memalloc(sizeof(t_thrprm));
}

static int		init(SDL_Window *win, SDL_Renderer **render)
{
	if (SDL_Init(SDL_INIT_VIDEO) >= 0)
	{
		if ((win = SDL_CreateWindow("RT | Launcher", 0, \
				SDL_WINDOWPOS_UNDEFINED, 800, 600, SDL_WINDOW_SHOWN)))
		{
			*render = SDL_CreateRenderer(win, -1,
					SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
			if (*render)
			{
				SDL_SetRenderDrawColor(*render, 0, 0, 0, 255);
				TTF_Init();
				if (IMG_Init(IMG_INIT_PNG) & IMG_INIT_PNG)
					return (1);
			}
		}
	}
	return (0);
}

void			launcher(char **scn, int nscn)
{
	t_launch	*launcher;
	t_texture	**textures;
	t_button	**ms_btns;

	!(launcher = ft_memalloc(sizeof(t_launch))) ? usage(0) : 0;
	!init(launcher->win, &(launcher->render)) ? usage(10) : 0;
	!(ms_btns = ft_memalloc(sizeof(t_button *) * (5 + 1))) ? usage(2) : 0;
	if (!(textures = textures_loader(4, launcher->render,
					"assets/button2.png", "assets/check.png", \
					"assets/right.png", "assets/left.png")))
		usage(3);
	SDL_SetRenderDrawColor(launcher->render, 0, 0, 0, 255);
	set(launcher);
	launcher->nb_scn = nscn;
	launcher->scn = scn;
	set_msbtns(launcher, ms_btns, textures);
	runner(launcher, ms_btns, textures);
	free_launcher(&launcher);
	btn_clean(ms_btns);
	delete_mtxtr(textures);
}
