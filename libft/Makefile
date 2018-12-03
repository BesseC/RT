NAME = libft.a

FLAGS := -Wall -Werror -Wextra
CC := gcc -g
DEPS = string list array gnl unicode mem print numbers maths printf

$(foreach dep, $(DEPS), $(eval include $(dep)/make.dep))
INCLUDES := $(addprefix -I, . $(DEPS) printf/includes)

OBJS := $(FILES:%.c=%.o)

all: $(NAME) $(DEPS)
	@printf "\033[1A\r\033[K""\r\033[K""\033[32m[LIBFT OK]\033[0m\n"

%.o: %.c
	@$(CC) -c $(INCLUDES) $(FLAGS) $< -o $@
	@printf "\033[1A\r\033[K""\r\033[K""\033[32m[LIBFT] \033[0m""Compilation de "$@"\n"

$(NAME): $(OBJS)
	@ar rc $(NAME) $(OBJS)
	@ranlib $(NAME)

clean:
	@rm -f $(OBJS)

fclean: clean
	@rm -f $(NAME)

re: fclean
	@make all

norminette:
	@norminette
