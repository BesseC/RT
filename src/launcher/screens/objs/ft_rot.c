/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_rot.c                                           :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: alerandy <alerandy@student.42.fr>          +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/06/07 11:16:19 by alerandy          #+#    #+#             */
/*   Updated: 2018/06/08 10:53:44 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include <math.h>

void	ft_rot_z(double *x, double *y, double teta)
{
	double	xtmp;

	xtmp = *x;
	*x = (xtmp * cos(teta)) - (*y * sin(teta));
	*y = (xtmp * sin(teta)) + (*y * cos(teta));
}

void	ft_rot_x(double *y, double *z, double teta)
{
	double	ztmp;

	ztmp = *z;
	*z = (sin(teta) * *y) + (cos(teta) * *z);
	*y = (cos(teta) * *y) - (sin(teta) * ztmp);
}

void	ft_rot_y(double *x, double *z, double teta)
{
	double	ztmp;

	ztmp = *z;
	*z = -(sin(teta) * *x) + (cos(teta) * *z);
	*x = (cos(teta) * *x) + (sin(teta) * ztmp);
}
