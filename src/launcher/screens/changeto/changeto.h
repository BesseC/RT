/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   changeto.h                                         :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: alerandy <alerandy@student.42.fr>          +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/05/04 05:41:08 by alerandy          #+#    #+#             */
/*   Updated: 2018/05/10 19:16:01 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef CHANGETO_H
# define CHANGETO_H

void				*to_newscreen(void *param);
void				*to_mainscreen(void *param);
void				*to_rtscreen(void *param);
void				*open_scn(void *param);
void				*to_rtopt(void *param);
void				*reset_resolution(void *param);
void				*change_resolution_min(void *param);
void				*change_resolution_max(void *param);

#endif
