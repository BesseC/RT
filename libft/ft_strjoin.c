/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_strjoin.c                                       :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: cbesse <marvin@42.fr>                      +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2017/11/10 14:54:57 by cbesse            #+#    #+#             */
/*   Updated: 2017/11/28 16:52:03 by cbesse           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "libft.h"

char	*ft_strjoin(char *s1, char const *s2)
{
	char	*str;

	if (s1 && s2)
	{
		if (!(str = (char *)malloc(sizeof(char)
						* (ft_strlen(s1) + ft_strlen(s2) + 1))))
			return (NULL);
		str = ft_strcpy(str, s1);
		str = ft_strcat(str, s2);
		return (str);
	}
	return (NULL);
}
