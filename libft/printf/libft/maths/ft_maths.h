/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_maths.h                                         :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/01/21 12:20:31 by scornaz           #+#    #+#             */
/*   Updated: 2018/01/24 11:29:01 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef FT_MATHS_H
# define FT_MATHS_H

# include <stdlib.h>
# include <stdio.h>
# include "../libft.h"

double	ft_sqrt(double nb);
double	ft_div(double a, double b);
double	ft_root(double nb, double root);
double	ft_pow2(double nb, double pow);
double	ft_pow(double base, double power);
double	ft_pow3(double base, double power);
int		is_prime(int nb);
void	prime_factors(int nb);
int		gcd(int a, int b);

#endif
