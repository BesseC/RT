/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_maths.h                                         :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/01/21 12:20:31 by scornaz           #+#    #+#             */
/*   Updated: 2018/04/01 16:17:44 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef FT_MATHS_H
# define FT_MATHS_H

# include <stdlib.h>
# include <stdio.h>
# include "../libft.h"

typedef struct	s_point
{
	int	x;
	int	y;
}				t_point;

double			ft_sqrt(double nb);
double			ft_div(double a, double b);
double			ft_root(double nb, double root);
double			ft_pow2(double nb, double pow);
int				ft_pow(int base, int power);
double			ft_pow3(double base, double power);
int				is_prime(int nb);
void			prime_factors(int nb);
int				gcd(int a, int b);

#endif
