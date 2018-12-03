NAME = libft.a

FLAGS := -Wall -Werror -Wextra
CC := gcc -g
FILES := ft_nbrsize.c ft_all_perms.c ft_atoi.c ft_fact.c ft_itoa.c  ft_rotate.c ft_swapchar.c ft_tolower.c ft_toupper.c  ft_max.c ft_abs.c

include string/make.dep
include list/make.dep
include array/make.dep
include gnl/make.dep
include unicode/make.dep
include mem/make.dep
include print/make.dep

OBJS := $(FILES:%.c=%.o)
INCLUDES := $(addprefix -I, . $(INCLUDES))

all: $(NAME)

$(NAME): $(OBJS)
	ar rc $(NAME) $(OBJS)
	ranlib $(NAME)

%.o: %.c
	$(CC) -c $(INCLUDES) $(FLAGS) $< -o $@

clean:
	rm -f $(OBJS)

fclean: clean
	rm -f $(NAME)

re: fclean
	make all
