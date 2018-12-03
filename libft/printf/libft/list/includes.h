/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   includes.h                                         :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: scornaz <marvin@42.fr>                     +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2018/01/21 16:47:04 by scornaz           #+#    #+#             */
/*   Updated: 2018/01/24 11:39:10 by scornaz          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef INCLUDES_H
# define INCLUDES_H

# include "../libft.h"

typedef struct	s_list
{
	void			*content;
	size_t			content_size;
	struct s_list	*next;
}				t_list;

void			ft_lstreduce(t_list *list,
							void (*f)(void *a, t_list *b, void *opts),
							void *val, void *opts);
t_list			*ft_lstnew(void const *content, size_t content_size);
t_list			*ft_cpyrev(t_list *orig);
void			ft_lstdelone(t_list **alst, void (*del)(void*, size_t));
void			ft_lstdel(t_list **alst, void (*del)(void *, size_t));
void			ft_lstadd(t_list **alst, t_list *new);
void			ft_lstaddlast(t_list **list, t_list *new);
void			ft_lstiter(t_list *lst, void (*f)(t_list *elem));
void			ft_lstiter2(t_list *lst, void (*f)(t_list *elem, void *args),
						void *mem);
t_list			*ft_lstmap(t_list *lst, t_list *(*f)(t_list *elem));

t_list			*ft_lstsort(t_list *list, int (*cmp)(t_list*, t_list*));
int				ft_cmp_strlen(t_list *one, t_list *two);
int				ft_cmp_str(t_list *one, t_list *two);
int				ft_cmp_size(t_list *one, t_list *two);
void			ft_lstprint(t_list *el);

#endif
