ROOT="../../"
LIBFT_PATH=../../libft

include make.dep
include ../objects/make.dep
include ../maths/make.dep
include ../maths/vec3f/make.dep

TEST="test/valgrind.c"

all:
	make -C $(LIBFT_PATH)
	gcc -g $(SANITIZE) $(TEST) $(BRACKETS_INCLUDE) -I$(LIBFT_PATH)/gnl \
	$(addprefix ../objects/, $(OBJECTS_SRCS)) $(LOCAL_INCLUDES) $(MATHS_ABS_SRCS) \
	$(VEC3F_ABS_SRCS) -L$(LIBFT_PATH) -lft -lm `sdl2-config --libs --cflags` -lSDL2_image

run: all
	./a.out

valgrind: all
	valgrind --leak-check=full --show-leak-kinds=all ./a.out

sanitize:
	$(eval, $(SANITIZE := -fsanitize=leak -fsanitize=address))
	make all
	ASAN_OPTIONS=detect_leaks=1 ./a.out

debug: all
	lldb ./a.out
