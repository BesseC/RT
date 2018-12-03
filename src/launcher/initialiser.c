/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   initialiser.c                                      :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: alerandy <alerandy@student.42.fr>          +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/05/01 16:14:37 by alerandy          #+#    #+#             */
/*   Updated: 2018/05/30 15:25:23 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "interface.h"

void		set_msbtns2(t_launch *launcher, t_button **buttons, \
		t_texture **textures)
{
	buttons[2] = button_new(35, 350, 175, 36);
	buttons[2]->func = &reset_resolution;
	buttons[2]->param = (void*)launcher;
	buttons[2]->texture = textures[1];
	buttons[2]->t = ttf_newb(launcher->render, "Reset", buttons[2], \
			"assets/28 Days Later.ttf");
	buttons[3] = button_new(235, 350, 175, 36);
	buttons[3]->func = &change_resolution_min;
	buttons[3]->param = (void*)launcher;
	buttons[3]->texture = textures[1];
	buttons[3]->t = ttf_newb(launcher->render, "800 X 600", buttons[3], \
			"assets/28 Days Later.ttf");
	buttons[4] = button_new(435, 350, 175, 36);
	buttons[4]->func = &change_resolution_max;
	buttons[4]->param = (void*)launcher;
	buttons[4]->texture = textures[1];
	buttons[4]->t = ttf_newb(launcher->render, "1280 X 720", buttons[4], \
			"assets/28 Days Later.ttf");
}

void		set_msbtns(t_launch *launcher, t_button **buttons, \
		t_texture **textures)
{
	int			i;

	launcher->img.w = 800;
	launcher->img.h = 600;
	buttons[0] = button_new(35, 500, 175, 36);
	buttons[0]->func = &to_newscreen;
	buttons[0]->param = (void*)launcher;
	buttons[0]->texture = textures[0];
	buttons[0]->t = ttf_newb(launcher->render, "Open RT", buttons[0], \
			"assets/28 Days Later.ttf");
	buttons[1] = button_new(300, 500, 175, 36);
	buttons[1]->func = &to_rtscreen;
	buttons[1]->param = (void*)launcher;
	buttons[1]->texture = textures[0];
	buttons[1]->t = ttf_newb(launcher->render, "RTS", buttons[1], \
			"assets/28 Days Later.ttf");
	set_msbtns2(launcher, buttons, textures);
	i = -1;
	while (++i < 5 * 4)
	{
		buttons[i % 5]->clips[i / 5] = (SDL_Rect){0, 36, 175, 36};
		buttons[i % 5]->clips[i / 5].y = 36 * (i / 5);
	}
}

void		set_newbtns2(t_launch *launcher, t_button **btns, t_texture **txtr)
{
	int		i;
	char	*tmp;

	!txtr[0] ? usage(31) : 0;
	i = -1;
	tmp = NULL;
	while (++i < launcher->nb_scn)
	{
		!(btns[i] = button_new(12 + ((i * 200) % (4 * 200)), 200 + (((i / 4) \
			* 50) % (7 * 50)), 175, 36)) ? usage(5) : 0;
		btns[i]->func = &open_scn;
		btns[i]->param = (void *)(launcher->scn[i]);
		btns[i]->texture = txtr[0];
		tmp = ft_strndup(launcher->scn[i] + 7, 10);
		btns[i]->t = ttf_newb(launcher->render, \
				tmp, btns[i], "assets/28 Days Later.ttf");
		ft_strdel(&tmp);
	}
}

void		set_newbtns3(t_launch *launcher, t_button **btns, t_texture **txtr)
{
	int		i;

	i = 8;
	!(btns[0] = button_new(12, 150, 175, 36)) ? usage(5) : 0;
	btns[0]->func = &to_mainscreen;
	btns[0]->id = 1;
	btns[0]->param = (void*)launcher;
	btns[0]->texture = txtr[0];
	btns[0]->t = ttf_newb(launcher->render, "Home", btns[0], \
			"assets/28 Days Later.ttf");
	!(btns[1] = button_new(212, 150, 175, 36)) ? usage(5) : 0;
	btns[1]->func = (void *)&i;
	btns[1]->id = 3;
	btns[1]->texture = txtr[0];
	btns[1]->t = ttf_newb(launcher->render, "Refresh", btns[1], \
			"assets/28 Days Later.ttf");
	!(btns[2] = button_new(35, 540, 59, 57)) ? usage(5) : 0;
	btns[2]->func = (void *)&i;
	btns[2]->id = 420;
	btns[2]->texture = txtr[3];
	!(btns[3] = button_new(696, 540, 59, 57)) ? usage(5) : 0;
	btns[3]->func = (void *)&i;
	btns[3]->id = 421;
	btns[3]->texture = txtr[2];
}

void		set_newbtns(t_launch *launcher, t_button **buttons, \
		t_texture **textures)
{
	int		i;
	int		j;

	set_newbtns3(launcher, buttons, textures);
	set_newbtns2(launcher, buttons + 4, textures);
	i = -1;
	j = launcher->nb_scn + 4;
	while (++i < j * 4)
	{
		buttons[i % j]->clips[i / j] = i % j == 2 || i % j == 3 ? \
					(SDL_Rect){0, 50, 59, 57} : (SDL_Rect){0, 36, 175, 36};
		buttons[i % j]->clips[i / j].y = i % j == 2 || i % j == 3 ? \
										57 * (i / j) : 36 * (i / j);
	}
}
