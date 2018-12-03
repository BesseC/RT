/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   itoas.c                                            :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/01/17 17:38:56 by scornaz           #+#    #+#             */
/*   Updated: 2018/05/09 17:44:38 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "printf.h"
#include <inttypes.h>

static char		*s_itoa(intmax_t v, int maj, t_num *a)
{
	char		*res;
	char		*tmp;
	int			i;
	uintmax_t	val;

	tmp = (char*)malloc(sizeof(char) * 65);
	if (v < 0)
		a->sign = -1;
	val = v < 0 ? -v : v;
	tmp[64] = '\0';
	i = 63;
	while ((int)val >= a->base)
	{
		tmp[i] = maj ?
			"0123456789ABCDEF"[val % a->base] :
			"0123456789abcdef"[val % a->base];
		val /= a->base;
		i--;
	}
	tmp[i] = maj ?
		"0123456789ABCDEF"[val % a->base] :
		"0123456789abcdef"[val % a->base];
	res = ft_strdup(tmp + i);
	free(tmp);
	return (res);
}

static char		*u_itoa(uintmax_t val, int maj, t_num *a)
{
	char	*res;
	char	*tmp;
	int		i;

	tmp = (char*)malloc(sizeof(char) * 65);
	tmp[64] = '\0';
	i = 63;
	while (val >= (uintmax_t)a->base)
	{
		tmp[i] = maj ?
			"0123456789ABCDEF"[val % a->base] :
			"0123456789abcdef"[val % a->base];
		val /= (uintmax_t)a->base;
		i--;
	}
	tmp[i] = maj ?
		"0123456789ABCDEF"[val % a->base] :
		"0123456789abcdef"[val % a->base];
	res = ft_strdup(tmp + i);
	free(tmp);
	return (res);
}

static void		cast_s(t_num *a, void *val)
{
	if (ft_strequ("hh", a->modifiers))
		a->value = s_itoa(*(char*)val, ft_strfind("DI", a->type), a);
	else if (ft_strequ("h", a->modifiers))
		a->value = s_itoa(*(short*)val, ft_strfind("DI", a->type), a);
	else if (ft_strequ("l", a->modifiers))
		a->value = s_itoa(*(long*)val, ft_strfind("DI", a->type), a);
	else if (ft_strequ("ll", a->modifiers))
		a->value = s_itoa(*(long long*)val, ft_strfind("DI", a->type), a);
	else if (ft_strequ("j", a->modifiers))
		a->value = s_itoa(*(intmax_t*)val, ft_strfind("DI", a->type), a);
	else if (ft_strequ("z", a->modifiers))
		a->value = s_itoa(*(ssize_t*)val, ft_strfind("DI", a->type), a);
	else
		a->value = s_itoa(*(int*)val, ft_strfind("DI", a->type), a);
}

static void		cast_u(t_num *a, void *val)
{
	if (ft_strequ("hh", a->modifiers))
		a->value = u_itoa(*(unsigned char*)val,
						ft_strfind("OUX", a->type), a);
	else if (ft_strequ("h", a->modifiers))
		a->value = u_itoa(*(unsigned short*)val,
						ft_strfind("OUX", a->type), a);
	else if (ft_strequ("l", a->modifiers))
		a->value = u_itoa(*(unsigned long*)val,
						ft_strfind("OUX", a->type), a);
	else if (ft_strequ("ll", a->modifiers))
		a->value = u_itoa(*(unsigned long long*)val,
						ft_strfind("OUX", a->type), a);
	else if (ft_strequ("j", a->modifiers))
		a->value = u_itoa(*(uintmax_t*)val,
						ft_strfind("OUX", a->type), a);
	else if (ft_strequ("z", a->modifiers))
		a->value = u_itoa(*(size_t*)val,
						ft_strfind("OUX", a->type), a);
	else
		a->value = u_itoa(*(unsigned int*)val,
						ft_strfind("OUX", a->type), a);
}

void			itoas(t_num *a, void *val)
{
	if (ft_strfind("diDI", a->type))
		cast_s(a, val);
	else if (ft_strfind("ouxOUXp", a->type))
		cast_u(a, val);
}
