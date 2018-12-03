/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   mini_ls.h                                          :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: alerandy <alerandy@student.42.fr>          +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/04/25 16:22:11 by alerandy          #+#    #+#             */
/*   Updated: 2018/05/12 01:22:39 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef MINI_LS_H
# define MINI_LS_H

# include "rtv1.h"
# include "interface.h"
# include <dirent.h>
# include <sys/types.h>
# include <sys/dir.h>
# include <sys/stat.h>

# define _DIRENT struct dirent

void		mini_ls(void);

#endif
