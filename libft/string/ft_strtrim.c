/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_strtrim.c                                       :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2017/11/09 14:08:37 by scornaz           #+#    #+#             */
/*   Updated: 2018/04/01 16:16:08 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "libft.h"

static int	is_blank(char c)
{
	return (c == ' ' || c == '\t' || c == '\n');
}

char		*ft_strtrim(char const *s)
{
	int		start;
	int		end;

	start = 0;
	end = 0;
	while (is_blank(s[start]))
		++start;
	while (s[end])
		++end;
	if (start == end)
		return (ft_strdup(""));
	while (is_blank(s[end - 1]))
		--end;
	return (ft_strsub(s, start, end - start));
}

char		*rm_char(char *str, char c)
{
	char	*ret;
	int		i;
	int		len;

	i = 0;
	len = ft_strlen(str);
	ret = malloc(len + 1);
	while (*str)
	{
		if (*str != c)
			ret[i++] = *str;
		++str;
	}
	free(str - len - 1);
	ret[i] = 0;
	return (ret);
}
