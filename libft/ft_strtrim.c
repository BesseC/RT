/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_strtrim.c                                       :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: cbesse <marvin@42.fr>                      +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2017/11/10 15:23:01 by cbesse            #+#    #+#             */
/*   Updated: 2017/11/13 15:16:21 by cbesse           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "libft.h"

char	*ft_strtrim(char const *s)
{
	int		i;
	int		len;
	char	*str;

	if (s)
	{
		i = -1;
		len = ft_strlen(s);
		while (s[len - 1] == ' ' || s[len - 1] == '\t' || s[len - 1] == '\n')
			len--;
		while (s[++i] == ' ' || s[i] == '\t' || s[i] == '\n')
			len--;
		if (len <= 0)
			len = 0;
		if (!(str = (char *)malloc(sizeof(char) * len + 1)))
			return (NULL);
		s = s + i;
		i = -1;
		while (++i < len)
			str[i] = *s++;
		str[i] = '\0';
		return (str);
	}
	return (NULL);
}
