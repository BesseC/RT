/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_strtrim.c                                       :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2017/11/09 14:08:37 by scornaz           #+#    #+#             */
/*   Updated: 2017/11/13 15:16:44 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "libft.h"

static int	is_blank(char c)
{
	return (c == ' ' || c == '\t' || c == '\n');
}

char		*ft_strtrim(char const *s)
{
	char	*copy;
	int		start;
	int		end;
	int		i;

	if (!(copy = (char*)s))
		return (NULL);
	start = -1;
	end = -1;
	while (is_blank(s[++start]))
		;
	while (copy[++end])
		;
	if (start == end)
		return (ft_strdup(""));
	while (is_blank(copy[--end]))
		;
	if (!(copy = (char*)malloc(sizeof(char) * (end - start))))
		return (NULL);
	s += start;
	i = 0;
	while (i <= end - start)
		copy[i++] = *s++;
	copy[i] = '\0';
	return (copy);
}
