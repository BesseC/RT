/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   rt.h                                               :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: cbesse <cbesse@student.42.fr>              +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/04/11 13:55:55 by cbesse            #+#    #+#             */
/*   Updated: 2018/06/09 18:26:02 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef RT_H
# define RT_H
# include "libft.h"
# include <math.h>
# include <float.h>
# include <sys/types.h>
# include <sys/stat.h>
# include <fcntl.h>
# include <complex.h>
# include "rt2.h"
# define W_LENGHT 1000
# define W_WIDTH 1000
# define EPSILON 0.0001
# define AMBIANT 0
# define MAX_DEPTH 10
# include "rtv1.h"

t_vecteur		v_add(t_vecteur v1, t_vecteur v2);
t_vecteur		v_less(t_vecteur v1, t_vecteur v2);
t_vecteur		v_cross(t_vecteur v1, t_vecteur v2);
double			v_dot(t_vecteur v1, t_vecteur v2);
double			v_norm(t_vecteur v);
t_vecteur		v_mult(t_vecteur v1, double i);
t_vecteur		v_negate(t_vecteur v1);
t_vecteur		v_div(t_vecteur v1, double i);
t_vecteur		v_set(double x, double y, double z);
t_vecteur		v_normalize(t_vecteur v);
int				my_key_funct(int keycode);
int				set_sphere(t_scene *scene, char **tab);
void			set_min_max(double min, double max, double *min_max);
int				hit_sphere(t_sphere2 *sphere, t_ray2 *ray,
		double *min_max, t_record *rec);
int				hit_cone(t_cone2 *cone, t_ray2 *ray,
		double *min_max, t_record *rec);
int				hit_cylindre(t_cylindre *cyl, t_ray2 *ray,
		double *min_max, t_record *rec);
int				hit_plan(t_plan *plan, t_ray2 *ray,
		double *min_max, t_record *rec);
int				hit_qqch(t_formlist *list, t_ray2 *ray,
		double *min_max, t_record *rec);
t_formlist		*set_list(void);
void			sphere_rec(t_ray2 *ray, double t, t_sphere2 *s, t_record *rec);
void			cyl_rec(t_ray2 *ray, double t, t_cylindre *cyl, t_record *rec);
void			cone_rec(t_ray2 *ray, double t, t_cone2 *cone, t_record *rec);
double			*cone_tab(t_cone2 *cone, t_ray2 *ray);
void			ret_inter(t_formlist list, double *min_max,
		t_record *rec, int *hit);
t_vecteur		libe(void **r, double *min_max, t_vecteur vr);
t_vecteur		r_color(t_ray2 *ray, t_scene scene, int depht);
void			printexit(void);
void			tab_free(char **tab, char *line);
int				set_plan(t_scene *scene, char **tab);
int				set_cylindre(t_scene *scene, char **tab);
int				set_cone(t_scene *scene, char **tab);
int				set_light(t_scene *scene, char **tab);
int				set_cam(t_scene *scene, char **tab);
void			ft_parseur(char **av, t_scene *scene);
void			shadow_ret_inter(t_formlist list, double *min_max, \
		t_record *rec, int *hit);
int				shadow_hit_qqch(t_formlist *list, t_ray2 *ray, \
		double *min_max, t_record *rec);
int				hit_fcylindre(t_fcylindre *fcyl, t_ray2 *ray, \
		double *min_max, t_record *rec);
int				hit_fcone(t_fcone *fcone, t_ray2 *ray, double *min_max, \
		t_record *rec);
void			ft_convert(t_obj **obj, t_scene *scene);
void			ft_raytrace(t_scene *scene, t_vec3f *color, t_vec3f ori, \
		t_vec3f dir);
void			fcylindre_def(t_obj *obj, t_scene *scene);
void			def_fcone(t_obj *obj, t_scene *scene);
void			free_scene(t_scene *scene);

void			light_def(t_sphere *sphere, t_scene *scene);
void			cone_def(t_obj *obj, t_scene *scene);
void			cylindre_def(t_obj *obj, t_scene *scene);
void			plan_def(t_obj *obj, t_scene *scene);
void			sphere_def(t_obj *obj, t_scene *scene);

t_vecteur		r_background(t_ray2 *ray);
t_vecteur		diffu_spec(t_vecteur light, t_record *r);
t_vecteur		c_shadow(t_vecteur *light, t_record *r, t_vecteur vr, \
		int n_light);
t_ray2			refraction(t_ray2 *ray, t_record *r);
float			ft_fresnel(t_ray2 *ray, t_record *r);
double			sqrt3(double a);
void			ft_equa3(double *abcd, _Complex double *res);
double			ft_real(_Complex double *res);
void			ft_equa2(t_equ4 *equa, double a, double b, double c);
void			ft_equa4(double *abcde, double *res);
void			tore_def(t_scene *scene);
int				hit_tore(t_tore *tore, t_ray2 *ray, double *min_max, \
	t_record *rec);
double			pattern(t_record *rec, unsigned type);
#endif
