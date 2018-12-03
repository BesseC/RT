all:
	@make -C ../libft
	gcc -g -I ../libft ../libft/print/ft_putnbr.c ft_itoa_base.c main.c && ./a.out

debug:
	lldb ./a.out

io:
	gcc -g get_inverse.c && ./a.out
