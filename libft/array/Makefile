LFT = -L.. -lft

include make.dep

all:
	make -C ..
	mv test test.c
	gcc -fsanitize=address -g -I. -I.. test.c $(LFT) ; mv test.c test

run: all
	./a.out

debug: all
	lldb ./a.out

valgrind: all
	valgrind ./a.out

norminette:
	norminette
