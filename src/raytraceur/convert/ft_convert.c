/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_convert.c                                       :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: alerandy <alerandy@student.42.fr>          +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/05/30 10:40:29 by alerandy          #+#    #+#             */
/*   Updated: 2018/06/09 19:56:21 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "rt.h"
#include "rtv1.h"

void	count_obj(t_obj **obj, t_scene *scene)
{
	int k;

	k = 0;
	while (obj[k])
	{
		if (ft_strequ(obj[k]->tag, "sphere"))
			scene->n_obj++;
		else if (ft_strequ(obj[k]->tag, "light"))
			scene->n_light++;
		else if (ft_strequ(obj[k]->tag, "cone"))
			scene->n_obj++;
		else if (ft_strequ(obj[k]->tag, "plane")
			|| ft_strequ(obj[k]->tag, "disque"))
			scene->n_obj++;
		else if (ft_strequ(obj[k]->tag, "cylinder"))
			scene->n_obj++;
		else if (ft_strequ(obj[k]->tag, "fcylinder"))
			scene->n_obj++;
		else if (ft_strequ(obj[k]->tag, "fcone"))
			scene->n_obj++;
		k++;
	}
}

void	ft_convert2(t_obj **obj, t_scene *scene, int i)
{
	if (ft_strequ(obj[i]->tag, "plane") || ft_strequ(obj[i]->tag, "disque"))
		plan_def(obj[i], scene);
	else if (ft_strequ(obj[i]->tag, "cylinder"))
		cylindre_def(obj[i], scene);
	else if (ft_strequ(obj[i]->tag, "fcylinder"))
		fcylindre_def(obj[i], scene);
	else if (ft_strequ(obj[i]->tag, "fcone"))
		def_fcone(obj[i], scene);
}

void	ft_convert(t_obj **obj, t_scene *scene)
{
	int i;

	i = -1;
	scene->n_light = 0;
	scene->n_obj = 0;
	count_obj(obj, scene);
	scene->list = (t_formlist*)ft_memalloc(scene->n_obj * sizeof(t_formlist));
	scene->light = (t_vecteur*)ft_memalloc(scene->n_light * sizeof(t_vecteur));
	scene->i = 0;
	scene->k = 0;
	while (obj[++i])
	{
		if (ft_strequ(obj[i]->tag, "sphere"))
			sphere_def(obj[i], scene);
		else if (ft_strequ(obj[i]->tag, "light"))
			light_def(obj[i]->obj.sphere, scene);
		else if (ft_strequ(obj[i]->tag, "cone"))
			cone_def(obj[i], scene);
		ft_convert2(obj, scene, i);
	}
}

int		get_index(t_obj **lst, t_record *rec)
{
	int i;

	i = 0;
	while ((ft_strequ(lst[i]->tag, "light")))
	{
		i++;
		rec->index++;
	}
	while (i < rec->index)
	{
		if (ft_strequ(lst[i]->tag, "light"))
		{
			i++;
			rec->index++;
		}
		i++;
	}
	if (ft_strequ(lst[i]->tag, "light"))
	{
		i++;
		rec->index++;
	}
	return (i);
}

t_obj	*ft_objtouche(t_obj **lst, t_ray *tmp)
{
	t_scene		*scene;
	t_ray2		ray;
	t_record	rec;
	double		*min_max;
	int			i;

	min_max = (double *)ft_memalloc(2 * sizeof(double));
	set_min_max(0.0, DBL_MAX, min_max);
	ray.ori = v_set(tmp->rayorig.x, tmp->rayorig.y, tmp->rayorig.z);
	ray.dir = v_set(tmp->raydir.x, tmp->raydir.y, tmp->raydir.z);
	scene = ft_memalloc(sizeof(t_scene));
	ft_convert(lst, scene);
	if (hit_qqch(scene->list, &ray, min_max, &rec))
	{
		i = get_index(lst, &rec);
		free_scene(scene);
		free(min_max);
		return (lst[i]);
	}
	free_scene(scene);
	free(min_max);
	return (NULL);
}
