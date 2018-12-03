SRCS = root.c power.c prime_dec.c gcd.c io.c
OBJS = $(SRCS:.c=.o)

all: $(OBJS)
	gcc -g $? ../*.o main.c

%.o : %.c
	gcc -g -c $< -o $@

run: all
	./a.out

test:	all
	./test

io:
	g++ io.c && ./a.out
