/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   test_get_file_comment.c                            :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/04/16 17:42:59 by scornaz           #+#    #+#             */
/*   Updated: 2018/05/09 13:53:47 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "printf.h"
#include "get_next_line.h"

int	main(void)
{
	printf("%s", get_file_comment("tests/io", '#'));
	printf("%s", get_file_comment("tests/io", '@'));
	printf("%s", get_file_comment("tests/test_comment", '#'));
	printf("%s", get_file_comment("tests/config4", '#'));
	return (0);
}
