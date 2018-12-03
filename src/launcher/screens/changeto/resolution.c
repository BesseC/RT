/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   resolution.c                                       :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: acourtin <acourtin@student.42.fr>          +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/05/09 14:02:07 by acourtin          #+#    #+#             */
/*   Updated: 2018/05/09 17:50:12 by acourtin         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "interface.h"

void		*reset_resolution(void *param)
{
	t_launch	*launcher;

	launcher = param;
	launcher->width = 0;
	launcher->height = 0;
	ttf_destroy(launcher->titleby[7]);
	ttf_destroy(launcher->titleby[8]);
	ttf_destroy(launcher->titleby[9]);
	launcher->titleby[8] = ttf_new(launcher->render, "", \
		"assets/28 Days Later.ttf", (t_pos){350, 300, 40});
	launcher->titleby[7] = ttf_new(launcher->render, "Par Defaut", \
		"assets/28 Days Later.ttf", (t_pos){250, 300, 40});
	launcher->titleby[9] = ttf_new(launcher->render, "", \
		"assets/28 Days Later.ttf", (t_pos){400, 300, 40});
	ft_putendl("Resolution par defaut");
	return (NULL);
}

void		*change_resolution_min(void *param)
{
	t_launch	*launcher;

	launcher = param;
	launcher->width = 800;
	launcher->height = 600;
	ttf_destroy(launcher->titleby[7]);
	ttf_destroy(launcher->titleby[8]);
	ttf_destroy(launcher->titleby[9]);
	launcher->titleby[8] = ttf_new(launcher->render, "X", \
		"assets/28 Days Later.ttf", (t_pos){350, 300, 40});
	launcher->titleby[7] = ttf_new(launcher->render, "800", \
		"assets/28 Days Later.ttf", (t_pos){250, 300, 40});
	launcher->titleby[9] = ttf_new(launcher->render, "600", \
		"assets/28 Days Later.ttf", (t_pos){400, 300, 40});
	ft_putstr("Nouvelle Resolution : ");
	ft_putnbr(launcher->width);
	ft_putstr(" X ");
	ft_putnbr(launcher->height);
	ft_putchar('\n');
	return (NULL);
}

void		*change_resolution_max(void *param)
{
	t_launch	*launcher;

	launcher = param;
	launcher->width = 1280;
	launcher->height = 720;
	ttf_destroy(launcher->titleby[7]);
	ttf_destroy(launcher->titleby[8]);
	ttf_destroy(launcher->titleby[9]);
	launcher->titleby[8] = ttf_new(launcher->render, "X", \
		"assets/28 Days Later.ttf", (t_pos){350, 300, 40});
	launcher->titleby[7] = ttf_new(launcher->render, "1280", \
		"assets/28 Days Later.ttf", (t_pos){250, 300, 40});
	launcher->titleby[9] = ttf_new(launcher->render, "720", \
		"assets/28 Days Later.ttf", (t_pos){400, 300, 40});
	ft_putstr("Nouvelle Resolution : ");
	ft_putnbr(launcher->width);
	ft_putstr(" X ");
	ft_putnbr(launcher->height);
	ft_putchar('\n');
	return (NULL);
}
