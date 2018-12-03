/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   globals.c                                          :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/04/11 18:25:38 by scornaz           #+#    #+#             */
/*   Updated: 2018/05/11 15:17:46 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "parser.h"

void	globals_set(t_globals *globals, char *prop, char *type, void *val)
{
	if (ft_strequ(prop, "size"))
	{
		globals->width = ((int*)val)[0];
		globals->height = ((int*)val)[1];
		if (globals->width < 0 || globals->width > 1600)
			globals->width = 2500;
		if (globals->height < 0 || globals->height > 1600)
			globals->height = 1380;
	}
	else if (ft_strequ(prop, "from"))
		globals->from = *(t_vec3f*)val;
	else if (ft_strequ(prop, "to"))
	{
		globals->to = *(t_vec3f*)val;
		vec3f_normalize(&globals->to);
	}
	else
		ft_printf("no way for %s in %s!\n", prop, type);
}

void	globals_print(t_globals *globals)
{
	ft_printf("global configuration :\n%d et %d: \nfrom & to:\n",
				globals->width,
				globals->height);
	vec3f_print(&globals->from);
	vec3f_print(&globals->to);
}
