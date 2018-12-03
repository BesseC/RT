/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   free_launcher.c                                    :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: alerandy <alerandy@student.42.fr>          +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/05/26 15:02:00 by alerandy          #+#    #+#             */
/*   Updated: 2018/05/26 15:49:47 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "interface.h"

void		free_launcher(t_launch **launcher)
{
	int		i;

	i = -1;
	while (++i < MAXTHREAD)
		free((*launcher)->prm[i]);
	free((*launcher)->prm);
	i = 0;
	while ((*launcher)->scn[i])
		free((*launcher)->scn[i++]);
	free((*launcher)->scn);
	free((*launcher));
	*launcher = NULL;
}
