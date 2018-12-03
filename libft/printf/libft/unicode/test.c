/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   test.c                                             :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/01/24 12:02:40 by scornaz           #+#    #+#             */
/*   Updated: 2018/01/25 14:27:39 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "unicode.h"
#include <stdio.h>

int	main(void)
{
	ft_putstr_utf8(L"☺️🙃\n");
	ft_putchar_utf8(L'☺');
	printf("%s", transform_utf8(L"\nco👱ucou les 😜 de ouf"));
	printf("%s", transform_utf8("🔭asd))"));
}
