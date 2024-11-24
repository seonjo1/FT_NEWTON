NAME :=	VULKAN_TUTORIAL

all: $(NAME)_debug

release: $(NAME)_release

$(NAME)_debug:
	@cmake -Bbuild -DCMAKE_BUILD_TYPE=Debug .
	@cmake --build build --config Debug
	@mv ./build/Debug/$(NAME).exe ./$(NAME)_debug.exe
	@echo [SUCCESS] $@ compiled successfully with debug mode and validation layers!

$(NAME)_release:
	@cmake -Bbuild -DCMAKE_BUILD_TYPE=Release .
	@cmake --build build --config Release
	@mv ./build/Release/$(NAME).exe ./$(NAME)_release.exe
	@echo [SUCCESS] $@ compiled successfully without validation layers!

clean:
	@rm -rf ./build/
	@echo [CLEAN] Build files have been removed!

fclean: clean
	@rm -rf $(NAME)_debug.exe $(NAME)_release.exe
	@echo [FCLEAN] Executable files have been fully removed!

re: fclean all

.PHONY: all clean fclean re debug release