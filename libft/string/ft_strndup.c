/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_strndup.c                                       :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: alerandy <marvin@42.fr>                    +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2017/11/15 22:19:59 by alerandy          #+#    #+#             */
/*   Updated: 2018/05/25 12:41:20 by alerandy         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "libft.h"

char	*ft_strndup(const char *s1, size_t n)
{
	char	*ptr;
	size_t	i;

	i = 0;
	if (ft_strlen(s1) < n)
		n = ft_strlen(s1);
	ptr = (char*)ft_memalloc(sizeof(char) * n + 1);
	if (ptr == 0)
		return (ptr);
	while (s1[i] && i < n)
	{
		ptr[i] = ((char*)s1)[i];
		i++;
	}
	return (ptr);
}
