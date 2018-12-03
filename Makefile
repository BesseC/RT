# **************************************************************************** #
#                                                                              #
#                                                         :::      ::::::::    #
#    Makefile                                           :+:      :+:    :+:    #
#                                                     +:+ +:+         +:+      #
#    By: alerandy <alerandy@student.42.fr>          +#+  +:+       +#+         #
#                                                 +#+#+#+#+#+   +#+            #
#    Created: 2018/04/17 16:41:43 by alerandy          #+#    #+#              #
#    Updated: 2018/06/11 11:32:10 by alerandy         ###   ########.fr        #
#                                                                              #
# **************************************************************************** #

include includes.dep

NAME=rt
SANITIZE = ""
CFLAGS = -Wall -Werror -Wextra
#CC = $(HOME)/.brew/Cellar/gcc/8.1.0/bin/gcc-8
CC = gcc
COMPILE=$(CC) -g3 $(SANITIZE) $(CFLAGS) #-O3

all: libft SDL2 $(NAME)
	@printf "\033[1A\r\033[K""\r\033[K""\033[32m[RT Compilé]\033[0m\n"

$(NAME): $(OBJS) $(HEADERS)
	@printf "\033[1A\r\033[K""\r\033[K""\033[32m[RT] \033[0m""Compilation""\n"
	@$(COMPILE) $(INCLUDE) $(PATH_OBJ) $(LINK) -o $(NAME)

sanitize:
	$(eval SANITIZE = -fsanitize=address)
	make all

# Vérifie si SDL2 existe, sinon l'installe.
SDL2:
	@./vendor/SDL/getsdl.sh

# Compilation des fichiers .c en les cherchant selon le VPATH.
%.o : %.c $(HEADERS)
	@mkdir -p $(OPATH)
	@mkdir -p screenshots
	@#$(COMPILE) $(INCLUDE) `cat .sdl_includes` -c $< -o $(OPATH)$@
	@printf "\033[1A\r\033[K""\r\033[K""\033[32m[RT] \033[0m""Compilation de "$@"\n"
	@$(COMPILE) $(INCLUDE) -c $< -o $(OPATH)$@

# Liste des rêgles de base d'un Makefile :
clean :
	@$(MAKE) -C libft clean
	@rm -rf $(OPATH)
	@echo "Librairie et ressources supprimés."

fclean : clean
	@$(MAKE) -C libft fclean
	@rm -rf $(NAME)
	@rm -rf screenshots
	@echo "RT Supprimé."

re :  fclean all
#END OF LIST

# Petite liste de Make pour la norm :
norm:
	@clear
	@echo "\033[33mScan des fichiers .h :\033[0m"
	@norminette $(shell find src -regex ".\{1,200\}\.h" | xargs) rtv1.h
	@echo "\n\033[33mScan des fichiers .c :\033[0m"
	@norminette $(ALLC)

normall: clean norm
	@make -C libft norminette

normlib: clean
	@make -C libft norminette
#END OF LIST

libft:
	@make -j -C libft

debug: all
	lldb ~/rtv1/rtv1 configs/config4

valgrind: all
	valgrind --leak-check=full ./$(NAME)

delib:
	#cd vendor/SDL && rm -rf ./SDL2-2.0.8/ ./SDL2-2.0.8.tar.gz ./SDL2_image-2.0.3/ ./SDL2_image-2.0.3.tar.gz ./SDL2_ttf-2.0.14/ ./SDL2_ttf-2.0.14.tar.gz lib include bin share

.PHONY : libft SDL2
