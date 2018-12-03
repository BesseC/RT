/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   merge.c                                            :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/01/09 15:13:32 by scornaz           #+#    #+#             */
/*   Updated: 2018/01/09 15:21:51 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

void	print(int *a, int len)
{
	int i;

	i = 0;
	while (i < len)
		printf("%d ", a[i++]);
	printf("\n");
}

int		*merge(int *one, int *two, int len)
{
	int *res;
	int i;

	i = 0;
	res = malloc(sizeof(int) * (len + 1));
	while (i < len)
	{
		if (*one == -1)
			res[i] = *two++;
		else if (*two == -1)
			res[i] = *one++;
		else if (*one < *two)
			res[i] = *one++;
		else
			res[i] = *two++;
		++i;
	}
	res[i] = -1;
	return (res);
}

int		*sort(int *array, int len)
{
	int *copy;

	if (!len)
		return (0);
	else if (len == 1)
	{
		copy = malloc(sizeof(int) * 2);
		copy[0] = array[0];
		copy[1] = -1;
	}
	else
		copy = merge(sort(array, len / 2),
				sort(array + (len / 2), len - len / 2),
				len);
	return (copy);
}
