/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   mainscreen.c                                       :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: alerandy <alerandy@student.42.fr>          +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/05/02 19:13:22 by alerandy          #+#    #+#             */
/*   Updated: 2018/06/06 13:08:04 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "interface.h"

int			is_triggered(t_button *btn)
{
	if (btn != NULL)
	{
		if (btn->trigger)
		{
			btn->trigger = 0;
			return (1);
		}
	}
	return (0);
}

static void	position_ttf(t_launch *launch, t_ttf **titleby, t_pos p)
{
	ttf_destroy(titleby[0]);
	titleby[0] = ttf_new(launch->render, "RT", "assets/docteur_atomic.ttf", p);
	titleby[1]->dstrect.y = titleby[2]->dstrect.y;
	titleby[1]->dstrect.x = titleby[2]->dstrect.x - titleby[1]->texw - 10;
	titleby[6]->dstrect.y = 300;
	titleby[6]->dstrect.x = 35;
	titleby[7]->dstrect.y = 300;
	titleby[7]->dstrect.x = 350;
	titleby[8]->dstrect.y = 300;
	titleby[8]->dstrect.x = 250;
	titleby[9]->dstrect.y = 300;
	titleby[9]->dstrect.x = 400;
}

t_ttf		**ttf_newscreen(t_launch *launch)
{
	t_ttf	**titleby;
	t_pos	p;

	titleby = NULL;
	p.x = 500;
	p.z = 40;
	p.y = 0;
	if (launch->width == 800)
		titleby = multi_ttf(10, launch->render, "assets/28 Days Later.ttf",
			p, "RT", "by", "scornaz", "cbesse", "acourtin", "alerandy",
			"Resolution", "X", "800", "600");
	else if (launch->width == 1280)
		titleby = multi_ttf(10, launch->render, "assets/28 Days Later.ttf",
			p, "RT", "by", "scornaz", "cbesse", "acourtin", "alerandy",
			"Resolution", "X", "1280", "720");
	else
		titleby = multi_ttf(10, launch->render, "assets/28 Days Later.ttf",
			p, "RT", "by", "scornaz", "cbesse", "acourtin", "alerandy",
			"Resolution", "", "Par Defaut", "");
	p.x = 50;
	p.y = -70;
	p.z = 300;
	position_ttf(launch, titleby, p);
	return (titleby);
}

void		tab_render(t_launch *launcher, t_ttf **texts)
{
	int		i;

	i = -1;
	while (texts[++i])
		SDL_RenderCopy(launcher->render, texts[i]->texture, NULL, \
				&(texts[i]->dstrect));
}

void		mainscreen(t_launch *launcher, t_button **buttons)
{
	int			i;

	launcher->titleby = ttf_newscreen(launcher);
	while (launcher->state == MSCREEN)
	{
		SDL_WaitEvent(&(launcher->event));
		getndestroy_rt(launcher);
		SDL_RenderFillRect(launcher->render, &(launcher->img));
		tab_render(launcher, launcher->titleby);
		i = -1;
		while (buttons[++i])
		{
			watch_btn(launcher, buttons[i]);
			if ((is_triggered(buttons[i])))
				buttons[i]->func(buttons[i]->param);
		}
		SDL_RenderPresent(launcher->render);
	}
	m_ttf_destroy(launcher->titleby);
}
