/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   couleur.c                                          :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: cbesse <marvin@42.fr>                      +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/03/30 14:02:00 by cbesse            #+#    #+#             */
/*   Updated: 2018/04/12 14:31:21 by cbesse           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "rt.h"
#define AMBIANT 0
#define MAX_DEPHT 10
#define EPSILON 0.0001
#define IOR 1.3

t_vecteur	r_background(t_ray *ray)
{
	t_vecteur	v2;
	t_vecteur	v1;
	t_vecteur	vr;
	double		t;

	v1 = v_set(1.0, 1.0, 1.0);
	v2 = v_set(0.5, 0.7, 1.0);
	t = 0.5 * (ray->dir.y + 1.0);
	vr = v_add(v_mult(v1, (1.0 - t)), v_mult(v2, t));
	return (vr);
}

t_vecteur	diffu_spec(t_vecteur light, t_record *r)
{
	t_vecteur	temp;
	t_vecteur	refrac;
	t_vecteur	diffu;
	double		shade;
	double		phong;

	temp = v_normalize(v_less(light, r->p));
	refrac = v_less(v_mult(r->normal, 2 * v_dot(r->normal, temp)), temp);
	phong = v_dot(refrac, temp);
	shade = v_dot(r->normal, temp);
	shade = shade < 0 ? 0 : shade;
	phong = phong < 0 ? 0 : phong;
	phong = pow(phong, 16);
	diffu.x = r->color.x * (AMBIANT + (1 - AMBIANT) * shade) + phong;
	diffu.y = r->color.y * (AMBIANT + (1 - AMBIANT) * shade) + phong;
	diffu.z = r->color.z * (AMBIANT + (1 - AMBIANT) * shade) + phong;
	return (diffu);
}

t_vecteur	c_shadow(t_vecteur *light, t_record *r, t_vecteur vr, int n_light)
{
	vr = v_add(diffu_spec(light[n_light], &r[0]), vr);
	vr.x = vr.x > 1 ? 1 : vr.x;
	vr.y = vr.y > 1 ? 1 : vr.y;
	vr.z = vr.z > 1 ? 1 : vr.z;
	return (vr);
}

t_ray	refraction(t_ray *ray, t_record *r)
{
	double		cosi;
	double		eta;
	double		k;
	t_ray			refrac;
	double etat = IOR;
	double etai = 1;

	cosi = v_dot(r->normal, v_normalize(ray->dir));
	if (cosi < 0)
		cosi = -cosi;
	else
		{
			r->normal.x = -r->normal.x;
			r->normal.y = -r->normal.y;
			r->normal.z = -r->normal.z;
			etai = 1;
			etai = IOR;
		}
	eta = etai / etat;
	k = 1 - eta * eta * (1 - cosi * cosi);
	if (k > 0)
	{
		refrac.dir = v_add(v_mult(ray->dir, eta), v_mult(r->normal, eta * cosi - sqrtf(k)));
	}
	else
		refrac.dir = v_set(0,0,0);
	refrac.dir = v_normalize(refrac.dir);
	refrac.ori = v_less(r->p, v_mult(r[0].normal, EPSILON));
	return(refrac);
}

float	ft_fresnel(t_ray *ray, t_record *r)
{

		double tmp;
		double kr;
		tmp = -v_dot(ray->dir, r[0].normal);
		tmp = pow(1 - tmp, 3);
		kr = 1 * 0.1 + tmp * (1 - 0.1);
		kr = kr > 1 ? 1 : kr;
		return(kr);

}


t_vecteur	r_color(t_ray *ray, t_scene scene, int depht)
{
	t_vecteur	vr;
	t_record	*r;
	double		*min_max;
	t_ray		sray;
	t_ray		reflec;
	t_ray		refrac;
	int i;
	t_vecteur tmp;
	float kr;
	t_vecteur refraccolor;
	t_vecteur refleccolor;


	i = scene.n_light;

	/*if(depht == 1)
		{
			printf("%f ", scene.light[0].y);
	}*/

	r = (t_record*)ft_memalloc(sizeof(t_record) * 2);
	min_max = (double *)ft_memalloc(2 * sizeof(double));
	set_min_max(0.0, DBL_MAX, min_max);
	vr = v_set(0, 0, 0);
	if (hit_qqch(scene.list, ray, min_max, &r[0]))
	{
		//vr = v_set(r[0].color.x, r[0].color.y, r[0].color.z);
		while (i-- > 0)
		{
			set_min_max(EPSILON, 1, min_max);
			sray.ori = v_add(r[0].p, v_mult(r[0].normal, EPSILON));
			sray.dir = v_less(scene.light[i], r[0].p);
			//printf("%f \n",v_norm(v_less(scene.light[i], r[0].p)));
			if (!(shadow_hit_qqch(scene.list, &sray, min_max, &r[1])))
				vr = c_shadow(scene.light, &r[0], vr, i);
			else
				if(r[1].kt > 0)
				{
					vr = c_shadow(scene.light, &r[0], vr, i);
					vr = v_div(vr, 1-r[1].kt + 1);
				}
		}
		if (depht < MAX_DEPHT)
		{
			if (r[0].ks == 1 && r[0].kt == 0)
			{
				reflec.ori = v_add(r[0].p, v_mult(r[0].normal, EPSILON));
				reflec.dir = v_less(ray->dir, v_mult(r[0].normal, 2 * v_dot(ray->dir, r[0].normal)));
				reflec.dir = v_normalize(reflec.dir);
				vr = r_color(&reflec, scene, depht + 1);
				vr.x = vr.x * r[0].color.x;
				vr.y = vr.y * r[0].color.y;
				vr.z = vr.z * r[0].color.z;
				vr.x = vr.x > 1 ? 1 : vr.x;
				vr.y = vr.y > 1 ? 1 : vr.y;
				vr.z = vr.z > 1 ? 1 : vr.z;
				/*vr = v_add(tmp, vr);
				vr.x = vr.x > 1 ? 1 : vr.x;
				vr.y = vr.y > 1 ? 1 : vr.y;
				vr.z = vr.z > 1 ? 1 : vr.z;
		*/	}
			if (r[0].kt > 0 && r[0].ks != 1)
			{
			//	printf("test\n");s
				refrac = refraction(ray, &(r[0]));
				vr = v_mult(r_color(&refrac, scene, depht + 1), r[0].kt);
				//vr = v_add(tmp, vr);
				//vr.x = vr.x > 1 ? 1 : vr.x;
				//vr.y = vr.y > 1 ? 1 : vr.y;
				//vr.z = vr.z > 1 ? 1 : vr.z;
				vr.x = vr.x * r[0].color.x;
				vr.y = vr.y * r[0].color.y;
				vr.z = vr.z * r[0].color.z;
				vr.x = vr.x > 1 ? 1 : vr.x;
				vr.y = vr.y > 1 ? 1 : vr.y;
				vr.z = vr.z > 1 ? 1 : vr.z;
			}
			if (r[0].kt > 0 && r[0].ks == 1)
			{
				kr = ft_fresnel(ray, &(r[0]));
				//printf("%f\n", kr);
				if (kr < 1)
				{
						refrac = refraction(ray, &(r[0]));
					//if (r[0].inside == 0)
							refrac.ori = v_less(r[0].p, v_mult(r[0].normal, EPSILON));
					//	else
						//	refrac.ori = v_add(r[0].p, v_mult(r[0].normal, EPSILON));
						refraccolor = v_mult(r_color(&refrac, scene, depht + 1), r[0].kt);
				}
				//if (r[0].inside == 0)
					reflec.ori = v_add(r[0].p, v_mult(r[0].normal, EPSILON));
				//else
				//	reflec.ori = v_less(r[0].p, v_mult(r[0].normal, EPSILON));
				reflec.dir = v_less(ray->dir, v_mult(r[0].normal, 2 * v_dot(ray->dir, r[0].normal)));
				reflec.dir = v_normalize(reflec.dir);
				refleccolor = r_color(&reflec, scene, depht + 1);
				vr = v_add(v_mult(refleccolor, kr), v_mult(refraccolor, (1 - kr)));

			//vr = v_mult(, vr);
				vr.x = vr.x * r[0].color.x;
				vr.y = vr.y * r[0].color.y;
				vr.z = vr.z * r[0].color.z;
			/*	vr.x = vr.x > 1 ? 1 : vr.x;
				vr.y = vr.y > 1 ? 1 : vr.y;
				vr.z = vr.z > 1 ? 1 : vr.z;
*/
			}
		}
		return (libe((void **)&r, (void **)&min_max, vr));
	}
	return (libe((void **)&r, (void **)&min_max, r_background(ray)));
}
