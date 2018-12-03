/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   test.c                                             :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/04/01 16:17:09 by scornaz           #+#    #+#             */
/*   Updated: 2018/04/01 16:17:11 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "unicode.h"
#include <stdio.h>

int	main(void)
{
	ft_putstr_utf8(L"☺️🙃\n");
	ft_putchar_utf8(L'☺');
	printf("%s", transform_utf8(L"👨‍👨‍👦co💑ucou les 😜 de ou\nf"));
	printf("%s", transform_utf8(L"🔭asd))"));
	printf("%s", transform_utf8(L"\nco👱ucou les 😜 de ouf"));
	printf("%s", transform_utf8("🔭asd))"));
}
