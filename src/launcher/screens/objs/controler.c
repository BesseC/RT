/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   controler.c                                        :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: alerandy <alerandy@student.42.fr>          +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/05/17 18:14:40 by alerandy          #+#    #+#             */
/*   Updated: 2018/05/31 15:24:34 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "l_obj.h"

static void	set(t_tfobj **tab)
{
	tab[0] = ft_memalloc(sizeof(t_tfobj));
	tab[0]->tag = ft_strdup("cylinder");
	tab[0]->func = &l_cyli;
	tab[1] = ft_memalloc(sizeof(t_tfobj));
	tab[1]->tag = ft_strdup("cone");
	tab[1]->func = &l_cone;
	tab[2] = ft_memalloc(sizeof(t_tfobj));
	tab[2]->tag = ft_strdup("plane");
	tab[2]->func = &l_plan;
	tab[3] = ft_memalloc(sizeof(t_tfobj));
	tab[3]->tag = ft_strdup("sphere");
	tab[3]->func = &l_sphe;
	tab[4] = ft_memalloc(sizeof(t_tfobj));
	tab[4]->tag = ft_strdup("fcone");
	tab[4]->func = &l_fcon;
	tab[5] = ft_memalloc(sizeof(t_tfobj));
	tab[5]->tag = ft_strdup("fcylinder");
	tab[5]->func = &l_fcyl;
	tab[6] = ft_memalloc(sizeof(t_tfobj));
	tab[6]->tag = ft_strdup("disque");
	tab[6]->func = &l_plan;
}

void		each_control(t_render_opts *opts, t_sdl *sdl, t_obj *obj)
{
	int		i;
	t_tfobj	**tab;

	(void)opts;
	tab = ft_memalloc(sizeof(t_tfobj *) * 8);
	set(tab);
	i = 0;
	while (tab[i] && !ft_strequ(tab[i]->tag, obj->tag))
		i++;
	tab[i] ? tab[i]->func(sdl, obj->obj, obj) : 0;
	i = -1;
	while (tab[++i])
	{
		free(tab[i]->tag);
		free(tab[i]);
	}
	free(tab);
}
