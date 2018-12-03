all:
	@make -C ..
	gcc -g -I.. -L.. -lft test.c unicode.c

run: all
	./a.out

test: all
	lldb ./a.out
