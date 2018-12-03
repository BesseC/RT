NAME := libftprintf.a
SRCS := itoas.c parse.c print.c strings.c ft_printf.c ft_sprintf.c change_types.c parser.c utils_printf.c
LIBFT_OBJS := $(addprefix libft/, ft_atoi.o ft_nbrsize.o print/ft_putchar.o print/ft_putstr.o string/ft_strany.o string/ft_strchr.o string/ft_strequ.o string/ft_strlen.o string/ft_strsub.o print/ft_isdigit.o print/ft_issep.o string/ft_strdup.o unicode/unicode.o mem/ft_bzero.o string/ft_strcmp.o array/array.o mem/ft_memcpy.o ft_abs.o unicode/cpy.o)
OBJS := $(SRCS:.c=.o)
INC := $(addprefix -I, libft includes)

all: $(NAME)

$(NAME): $(OBJS)
	 @make -C libft
	 @ar cr $(NAME) $(OBJS) $(LIBFT_OBJS)
	 @ranlib $(NAME)

%.o : %.c
	@gcc -c -g $(INC) $< -o $@

clean:
	@rm -rf $(OBJS)

fclean: clean
	@rm -rf $(NAME)

re: fclean all
