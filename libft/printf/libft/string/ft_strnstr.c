/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   strnstr.c                                          :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2017/11/09 10:45:30 by scornaz           #+#    #+#             */
/*   Updated: 2017/11/12 17:19:18 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include <string.h>
#include <unistd.h>

char	*ft_strnstr(const char *haystack, const char *needle, size_t len)
{
	unsigned int	i;
	unsigned int	j;

	i = 0;
	j = 0;
	if (!needle[0])
		return ((char*)haystack);
	while (i < len && haystack[i])
	{
		while (haystack[j + i] == needle[j])
		{
			if (j + i < len && !needle[j + 1])
				return ((char*)haystack + i);
			j++;
		}
		i++;
		j = 0;
	}
	return (NULL);
}
