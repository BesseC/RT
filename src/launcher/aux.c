/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   aux.c                                              :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: alerandy <alerandy@student.42.fr>          +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/05/12 08:42:34 by alerandy          #+#    #+#             */
/*   Updated: 2018/05/25 17:47:56 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "interface.h"

void		filter_chng(t_thrprm *param, t_button *btn)
{
	param->sdl->filter = btn->id - 1;
	param->sdl->is_rendering = 0;
	btn->trigger = 0;
}

void		btn_clean(t_button **btns)
{
	int		i;

	i = -1;
	if (btns)
	{
		while (btns[++i])
		{
			btns[i]->t ? ttf_destroy(btns[i]->t) : 0;
			free(btns[i]);
		}
		free(btns);
	}
}

void		destroy_thrprm(t_thrprm *prm)
{
	prm->event = NULL;
	ft_strdel(&prm->scn);
	prm->quited = 0;
	prm->sdl = NULL;
	prm->opts = NULL;
}

void		fill_thrprm(t_thrprm *prm, t_launch *launcher, t_button *btn)
{
	prm->event = &(launcher->event);
	prm->scn = ft_strdup(btn->param);
	prm->quited = 0;
	prm->sdl = NULL;
	prm->opts = NULL;
	prm->width = launcher->width;
	prm->height = launcher->height;
}

void		delete_mtxtr(t_texture **txtr)
{
	int		i;

	i = 0;
	while (txtr[i])
	{
		SDL_DestroyTexture(txtr[i]->texture);
		free(txtr[i++]);
	}
	free(txtr);
}
