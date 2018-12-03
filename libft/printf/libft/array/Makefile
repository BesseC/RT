LFT = -L.. -lft

include make.dep

all:
	#mv test test.c
	gcc -fsanitize=address -g $(LFT) $(FFILES) test.c #; mv test.c test

run: all
	./a.out

debug: all
	lldb ./a.out
