/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   interface.h                                        :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: alerandy <alerandy@student.42.fr>          +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/04/26 16:47:35 by alerandy          #+#    #+#             */
/*   Updated: 2018/06/09 15:04:07 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef INTERFACE_H
# define INTERFACE_H

# include "rtv1.h"
# include "sdl_mouse.h"
# include "changeto.h"
# include "mini_ls.h"

# define MAXTHREAD 5
# define ASPEED 0.1
# define LTIMEOUT 133

typedef enum		e_estate
{
	MSCREEN,
	OPTS,
	NEW,
	RTS,
	QUIT
}					t_estate;

typedef struct		s_thrparam
{
	char			*scn;
	SDL_Event		*event;
	int				quited;
	t_render_opts	*opts;
	t_sdl			*sdl;
	int				width;
	int				height;
	t_obj			*sobj;
}					t_thrprm;

typedef struct		s_launcher
{
	SDL_Event		event;
	SDL_Renderer	*render;
	SDL_Window		*win;
	SDL_Rect		img;
	pthread_t		thr[MAXTHREAD];
	t_thrprm		**prm;
	int				nb_scn;
	char			**scn;
	int				quit;
	t_estate		state;
	int				width;
	int				height;
	t_ttf			**titleby;
}					t_launch;

typedef struct		s_newscreen
{
	t_launch		*launcher;
	t_estate		newstate;
}					t_newscreen;

typedef struct		s_rt
{
	t_launch		*launcher;
	t_thrprm		*thr;
	t_obj			*selected;
}					t_rt;

typedef struct		s_tabfunc_obj
{
	char			*tag;
	int				id;
	void			(*func)();
}					t_tfobj;

void				set_newbtns(t_launch *launcher, t_button **buttons, \
		t_texture **textures);
void				set_msbtns(t_launch *launcher, t_button **buttons, \
		t_texture **textures);
void				set_msbtns2(t_launch *launcher, t_button **buttons, \
		t_texture **textures);

void				watch_btn(t_launch *launcher, t_button *buttons);
int					is_triggered(t_button *btn);
void				tab_render(t_launch *launcher, t_ttf **texts);

void				launcher(char **scn, int nscn);
void				usage(int err);
void				through_argv(t_thrprm *param);

void				new_rt(t_launch *launcher, t_texture **txtr);
void				mainscreen(t_launch *launcher, t_button **buttons);
void				rtscreen(t_launch *launcher, t_texture **txtr);
void				rt_opts(t_rt *opts);
void				getobj(t_rt *opts);
void				load_sdl(t_launch *launcher, t_ttf *load, t_texture **txtr);

t_ttf				**multi_ttf(int nb, SDL_Renderer *renderer, char *font, \
		t_pos pos, ...);
t_ttf				**ttf_newscreen(t_launch *launcher);
void				m_ttf_destroy(t_ttf **text);
void				delete_mtxtr(t_texture **txtr);

void				destroy_thrprm(t_thrprm *prm);
void				fill_thrprm(t_thrprm *prm, t_launch *launcher, \
		t_button *btn);
void				init_sdl(t_render_opts *opts, t_thrprm *param);
void				check_event(t_render_opts *opts, t_sdl *sdl, t_thrprm *prm);

t_button			**refresh_newbtns(t_launch *launcher, t_button **btns);
void				refresh_ls(t_launch *launcher);

void				btn_clean(t_button **btns);
void				obj_key(t_render_opts *opts, t_sdl *sdl, t_obj *obj);

void				each_control(t_render_opts *opts, t_sdl *sdl, t_obj *obj);
void				filter_chng(t_thrprm *param, t_button *btn);

t_obj				*ft_objtouche(t_obj **lst, t_ray *tmp);
void				free_launcher(t_launch **launcher);

void				filters_names(char *filter[]);

void				snap_screen(t_render_opts *opts, t_sdl *sdl);

void				getndestroy_rt(t_launch *launcher);

void				ft_rot_y(double *x, double *z, double a);
void				ft_rot_x(double *y, double *z, double a);
void				ft_rot_z(double *x, double *y, double a);

#endif
