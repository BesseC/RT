SRCS = main.c test_sort.c ft_lstadd.c ft_lstiter.c ft_lstmap.c ft_lstnew.c ft_lstdelone.c ft_lstdel.c ft_lstcpy.c ft_lstreduce.c 

all:
	gcc -g3 ../*.o $(SRCS)

do: all
	./a.out

libft:
	cd .. && make
	gcc ../*.o *.o testLibft.c && ./a.out
