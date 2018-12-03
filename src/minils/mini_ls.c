/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   mini_ls.c                                          :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: alerandy <alerandy@student.42.fr>          +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/04/25 16:19:45 by alerandy          #+#    #+#             */
/*   Updated: 2018/05/12 01:23:07 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "mini_ls.h"

int		count_file(void)
{
	_DIRENT	*fold;
	int		i;
	DIR		*dir;

	dir = NULL;
	fold = NULL;
	i = 0;
	if (!(dir = opendir("scenes")))
	{
		ft_putendl("Une erreur est survenue lors de l'ouverture des scènes.");
		exit(0);
	}
	else
	{
		while ((fold = readdir(dir)))
		{
			if (fold->d_name[0] != '.')
				i++;
		}
	}
	closedir(dir);
	return (i);
}

void	refresh_ls(t_launch *launcher)
{
	int		i;
	DIR		*dir;
	_DIRENT	*fold;

	i = 0;
	launcher->nb_scn = count_file();
	while (launcher->scn[i])
		ft_strdel(&launcher->scn[i++]);
	free(launcher->scn);
	if (!(dir = opendir("scenes")))
		ft_putendl("Une erreur est survenue lors de l'ouverture des scènes.");
	else if ((launcher->scn = ft_memalloc(sizeof(char*) * \
					(launcher->nb_scn + 1))))
	{
		i = 0;
		while ((fold = readdir(dir)))
			if (fold->d_name[0] != '.')
				launcher->scn[i++] = ft_strjoin("scenes/", fold->d_name);
		closedir(dir);
	}
	else
		usage(8);
}

void	mini_ls(void)
{
	int		i;
	DIR		*dir;
	_DIRENT	*fold;
	char	**scn;
	int		file;

	dir = NULL;
	fold = NULL;
	scn = NULL;
	i = 0;
	file = count_file();
	if (!(dir = opendir("scenes")))
		ft_putendl("Une erreur est survenue lors de l'ouverture des scènes.");
	else
	{
		if ((scn = ft_memalloc(sizeof(char*) * (file + 1))))
		{
			while ((fold = readdir(dir)))
				if (fold->d_name[0] != '.')
					scn[i++] = ft_strjoin("scenes/", fold->d_name);
			closedir(dir);
			launcher(scn, file);
		}
	}
}
