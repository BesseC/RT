/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_strmap.c                                        :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: cbesse <marvin@42.fr>                      +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2017/11/10 13:32:12 by cbesse            #+#    #+#             */
/*   Updated: 2017/11/13 15:11:14 by cbesse           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "libft.h"

char	*ft_strmap(char const *s, char (*f)(char))
{
	size_t	len;
	size_t	i;
	char	*ret;

	if (s)
	{
		i = 0;
		len = ft_strlen(s);
		if (!(ret = (char *)malloc(sizeof(char) * len + 1)))
			return (NULL);
		while (s[i])
		{
			ret[i] = f(s[i]);
			i++;
		}
		ret[i] = '\0';
		return (ret);
	}
	return (NULL);
}
