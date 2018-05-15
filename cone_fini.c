/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   cone_fini.c                                        :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: cbesse <marvin@42.fr>                      +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/05/10 19:07:50 by cbesse            #+#    #+#             */
/*   Updated: 2018/05/10 19:07:51 by cbesse           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "rt.h"

void	fcone_rec(t_ray *ray, double t, t_fcone *cone, t_record *rec)
{
	double		uv;
	t_vecteur	temp;
	t_vecteur	oc;

	rec->t = t;
	rec->p = v_add(ray->ori, v_mult(ray->dir, rec->t));
	oc = v_less(rec->p, cone->apex);
	if (v_dot(v_normalize(cone->dir), v_normalize(oc)) > 0)
		temp = v_set(cone->dir.x, cone->dir.y, cone->dir.z);
	else
		temp = v_set(-cone->dir.x, -cone->dir.y, -cone->dir.z);
	uv = v_norm(oc) / cos(cone->angle / 2);
	rec->normal = v_normalize(v_less(oc, v_mult(temp, uv)));
	if (v_dot(ray->dir, rec->normal) > 0)
	{
		//rec->normal.x = -rec->normal.x;
		//rec->normal.y = -rec->normal.y;
		//rec->normal.z = -rec->normal.z;
		rec->inside = 1;
	}
}

double	fconepv(t_ray *ray, t_fcone *fcone, t_record *rec, double t)
{
	double uv;
  t_vecteur p;
  t_vecteur temp;
  t_vecteur normal;
	t_vecteur pv;
	//t_vecteur napex;
	t_vecteur oc;
	double k;
	k = fcone->angle / 2;



	p = v_add(ray->ori, v_mult(ray->dir, t));
	//napex = v_add(fcone->apex, v_mult(fcone->dir, fcone->mid));
	oc = v_less(p, fcone->apex);
//	if (v_dot(v_normalize(fcone->dir), v_normalize(oc)) > 0)
//		temp = v_set(fcone->dir.x, fcone->dir.y, fcone->dir.z);
//	else
//		temp = v_set(-fcone->dir.x, -fcone->dir.y, -fcone->dir.z);
	uv = v_norm(oc) * cos(k);
	//temp = v_normalize(v_less(oc, v_mult(temp, uv)));
//	uv = sin(M_PI/2 - k) * uv;
	//uv = v_norm(v_less(p, napex)) * sin(k);
	//temp = v_set(-temp.x, -temp.y, -temp.z);
	//pv = v_less(p, v_mult(temp, uv));
	//printf("%f\n",uv );
/*	if (pv.x == 0)
	{
		printf("%f\n",uv );
		printf("pv = %f %f %f\n", pv.x, pv.y, pv.z);
		printf("p = %f %f %f\n", p.x, p.y, p.z);
		printf("normal = %f %f %f\n", -temp.x, -temp.y, -temp.z);
	}
	*/return(uv);
}

int fcone_test(t_ray *ray, t_fcone *fcone, t_record *rec, double t)
{

	t_vecteur pv;
  double ok;
	t_vecteur napex;
	t_vecteur p;


	p = v_add(ray->ori, v_mult(ray->dir, t));
	//napex = v_add(fcone->apex, v_mult(fcone->dir, fcone->mid));

	ok = fconepv(ray, fcone, rec, t);
	//napex = v_add(fcone->apex, v_mult(fcone->dir, fcone->mid));
	//ok = v_norm(v_less(pv, fcone->apex));
	//printf("napex = %f %f %f\n", napex.x, napex.y, napex.z);
	//printf("ok = %f \n", ok);
	//printf("pv = %f %f %f\n", pv.x, pv.y, pv.z);
	ok = v_dot(fcone->dir, v_normalize(v_less(p, fcone->apex))) < 0 ? -ok : ok;
	if (ok <= fcone->mid + fcone->size/2 && ok >= fcone->mid - fcone->size/2)
  {
		//printf("pv = %f %f %f\n", pv.x, pv.y, pv.z);
		//if (ok <= fcone->size/2)
  //  printf("%f\n", ok);
    return(1);
  }
  return(0);
}

int	hit_fconebord(t_fcone *fcone, t_ray *ray, double *min_max, t_record *rec, int f)
{
	//t_plan *plan1;
	//t_plan *plan2;
	int t;
	int p;
  /*t_vecteur h1;
  t_vecteur h2;

  h1 = v_mult(fcone->dir, fcone->mid + fcone->size);
  h2 = v_mult(fcone->dir, fcone->mid - fcone->size);
	//printf("h1 = %f\n", v_norm(h1));
	//printf("h2 = %f\n", v_norm(h2));
	plan1 = (t_plan *)ft_memalloc(sizeof(t_plan));
	plan2 = (t_plan *)ft_memalloc(sizeof(t_plan));
	plan1->point = v_add(fcone->apex, v_mult(fcone->dir, fcone->mid + fcone->size));
	plan2->point = v_add(fcone->apex, v_mult(fcone->dir, fcone->mid - fcone->size));
	plan1->vdir = v_set(fcone->dir.x, fcone->dir.y, fcone->dir.z);
	plan2->vdir = v_set(-fcone->dir.x, -fcone->dir.y, -fcone->dir.z);
	plan1->size = sqrt(fabs(pow(v_norm(h1),2) - pow(v_norm(h1) / cos(fcone->angle/2),2)));
	plan2->size = sqrt(fabs(pow(v_norm(h2),2) - pow(v_norm(h2) / cos(fcone->angle/2),2)));
  //printf("%f\n", plan2->size );*/
  if (fcone->plan1->size != 0)
		if(f == 1 && (t = hit_plan(fcone->plan1, ray, min_max, rec)))
		{
			set_min_max(min_max[0], rec->t, min_max);
			//ft_memdel((void **)&plan1);
			//ft_memdel((void **)&plan2);
			return (t);
		}
  if (fcone->plan2->size != 0)
		if(f == 2 && (p = hit_plan(fcone->plan2, ray, min_max, rec)))
		{
			set_min_max(min_max[0], rec->t, min_max);
			//ft_memdel((void **)&plan1);
			//ft_memdel((void **)&plan2);
			return (p);
		}
	//ft_memdel((void **)&plan1);
	//ft_memdel((void **)&plan2);
	return(0);
}

int	hit_fcone(t_fcone *fcone, t_ray *ray, double *min_max, t_record *rec)
{
  double a;
  double b;
  double c;
  double disc;
  double r;
  double k;
  t_vecteur x;

  k = tan(fcone->angle / 2);
  k = k * k;
  x = v_less(ray->ori, fcone->apex);
  a = v_dot(ray->dir, ray->dir) - (1 + k) * pow(v_dot(ray->dir, fcone->dir), 2);
  b = 2 * (v_dot(ray->dir, x) - (1 + k) * v_dot(ray->dir, fcone->dir) * v_dot(x, fcone->dir));
  c = v_dot(x, x) - (1 + k) * (pow(v_dot(x, fcone->dir), 2));
  disc = b * b - 4 * a * c;
  r = (-1 * b - sqrt(disc)) / (2 * a);
  if (r < min_max[1] && r > min_max[0] && fcone_test(ray, fcone, rec, r) == 1)
  {
    fcone_rec(ray, r, fcone, rec);
    return (1);
  }
  if (hit_fconebord(fcone, ray, min_max, rec, 1))
      return(1);
  if (hit_fconebord(fcone, ray, min_max, rec, 2))
    return(1);
  r = (-1 * b + sqrt(disc)) / (2 * a);
  if (r < min_max[1] && r > min_max[0] && fcone_test(ray, fcone, rec, r) == 1)
  {
    fcone_rec(ray, r, fcone, rec);
    return (1);
  }
  return(0);

}


void	attr_fcone(t_fcone *fcone, char **tab)
{
	t_vecteur h1;
	t_vecteur h2;

	tab[3][ft_strlen(tab[3]) - 1] = '\0';
	tab[7][ft_strlen(tab[7]) - 1] = '\0';
	fcone->apex.x = ft_atof(tab[1] + 1);
	fcone->apex.y = ft_atof(tab[2]);
	fcone->apex.z = ft_atof(tab[3]);
	fcone->angle = ft_atof(tab[4]) * M_PI / 180;
	fcone->dir.x = ft_atof(tab[5] + 1);
	fcone->dir.y = ft_atof(tab[6]);
	fcone->dir.z = ft_atof(tab[7]);
	fcone->dir = v_normalize(fcone->dir);
  fcone->size = ft_atof(tab[8]);
	fcone->mid = 0.3;
  h1 = v_mult(fcone->dir, fcone->mid + fcone->size/2);
  h2 = v_mult(fcone->dir, fcone->mid - fcone->size/2);
	//printf("h1 = %f\n", v_norm(h1));
	//printf("h2 = %f\n", v_norm(h2));
	fcone->plan1 = (t_plan *)ft_memalloc(sizeof(t_plan));
	fcone->plan2 = (t_plan *)ft_memalloc(sizeof(t_plan));
	fcone->plan1->point = v_add(fcone->apex, v_mult(fcone->dir, fcone->mid + fcone->size/2));
	fcone->plan2->point = v_add(fcone->apex, v_mult(fcone->dir, fcone->mid - fcone->size/2));
	fcone->plan1->vdir = v_set(fcone->dir.x, fcone->dir.y, fcone->dir.z);
	fcone->plan2->vdir = v_set(-fcone->dir.x, -fcone->dir.y, -fcone->dir.z);
	fcone->plan1->size = sqrt(fabs(pow(v_norm(h1),2) - pow(v_norm(h1) / cos(fcone->angle/2),2)));
	fcone->plan2->size = sqrt(fabs(pow(v_norm(h2),2) - pow(v_norm(h2) / cos(fcone->angle/2),2)));
}

int		set_fcone(t_scene *scene, char **tab)
{
	int	j;

	j = 0;
	while (tab[j++])
		;
	if (j - 1 != 14)
		return (-1);
	scene->list[scene->i].form = (t_fcone *)ft_memalloc(sizeof(t_fcone));
	attr_fcone(scene->list[scene->i].form, tab);
	scene->list[scene->i].color.x = ft_atof(tab[9] + 1);
	scene->list[scene->i].color.y = ft_atof(tab[10]);
	tab[11][ft_strlen(tab[11]) - 1] = '\0';
	scene->list[scene->i].color.z = ft_atof(tab[11]);
	scene->list[scene->i].ks = ft_atof(tab[12]);
	scene->list[scene->i].kt = ft_atof(tab[13]);
	scene->list[scene->i].type = 6;
	scene->i++;
	j = 0;
	while (j < scene->i)
		scene->list[j++].size = scene->i;
	return (1);
}
