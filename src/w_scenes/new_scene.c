/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   new_scene.c                                        :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: alerandy <alerandy@student.42.fr>          +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/06/10 13:52:29 by alerandy          #+#    #+#             */
/*   Updated: 2018/06/11 09:44:45 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "rtv1.h"
#include "w_write.h"

static void	w_cam(t_render_opts *opts, int fd)
{
	ft_putstr_fd("\nglobal (\n\tcamera (from (", fd);
	ft_putfloat_fd(opts->camorig.x, 3, fd);
	ft_putstr_fd(" ", fd);
	ft_putfloat_fd(opts->camorig.y, 3, fd);
	ft_putstr_fd(" ", fd);
	ft_putfloat_fd(opts->camorig.z, 3, fd);
	ft_putstr_fd(")) (to (", fd);
	ft_putfloat_fd(opts->camdir.x, 3, fd);
	ft_putstr_fd(" ", fd);
	ft_putfloat_fd(opts->camdir.y, 3, fd);
	ft_putstr_fd(" ", fd);
	ft_putfloat_fd(opts->camdir.z, 3, fd);
	ft_putstr_fd("))\n)", fd);
}

static void	object_write(t_obj *obj, int fd)
{
	ft_putstr_fd("\t", fd);
	ft_putstr_fd(obj->tag, fd);
	ft_putstr_fd(" (position (", fd);
	ft_putfloat_fd(obj->position.x, 3, fd);
	ft_putstr_fd(" ", fd);
	ft_putfloat_fd(obj->position.y, 3, fd);
	ft_putstr_fd(" ", fd);
	ft_putfloat_fd(obj->position.z, 3, fd);
	ft_putstr_fd(")) ", fd);
	w_obj(obj, fd);
	w_obj2(obj, fd);
	w_obj3(obj, fd);
	w_obj4(obj, fd);
	w_obj5(obj, fd);
	w_obj6(obj, fd);
}

static char	*getaname(char *str)
{
	char	*tmp;
	char	*tmp2;
	char	*tmp3;
	int		i;
	int		fd;

	i = 2;
	tmp = ft_strjoin(str, "V");
	tmp2 = ft_itoa(i);
	tmp3 = ft_strjoin(tmp, tmp2);
	while ((fd = open(tmp3, O_EXCL) > 0))
	{
		++i;
		free(tmp2);
		free(tmp3);
		tmp2 = ft_itoa(i);
		tmp3 = ft_strjoin(tmp, tmp2);
	}
	close(fd);
	free(tmp2);
	free(tmp);
	return (tmp3);
}

static void	w_objects(t_render_opts *opts, int fd)
{
	t_obj	**obj;
	int		i;

	obj = *(opts->spheres);
	i = -1;
	ft_putstr("Sauvegarde de : ");
	ft_putendl(opts->scname);
	ft_putstr_fd("objects (\n", fd);
	while (obj[++i])
	{
		object_write(obj[i], fd);
		ft_putendl_fd("", fd);
	}
}

int			new_scene(t_render_opts *opts)
{
	int		fd;
	mode_t	mode;
	char	*name;

	mode = S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH;
	name = getaname(opts->scname);
	if ((fd = open(name, O_RDWR | O_CREAT | O_APPEND, mode)) < 0)
	{
		free(name);
		ft_putendl("Saving new scene failed.");
		return (-1);
	}
	free(name);
	w_objects(opts, fd);
	ft_putendl_fd(")", fd);
	w_cam(opts, fd);
	close(fd);
	return (1);
}
