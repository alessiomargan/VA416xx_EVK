# What OS am I running on?
ifdef OS # Windows
    define remove_dir
        @if exist $(1) rmdir /s/q $(1)
    endef
else     # Linux
    ifeq ($(shell uname), Linux)
        define remove_dir
            @rm -rf $(1)
        endef
    endif
endif

# Optimization level
opt = Os

# Builds
init:
	$(call remove_dir, build)
	@mkdir build
	@cmake -E chdir build cmake -G "Unix Makefiles" -DOPT=$(opt) ..

clean:
	@echo Removing build files
	@make -C build clean

blinky:
	@make -C build blinky

ft_vor:
	@make -C build ft_vor

ft_vor_verbose:
	@make -C build VERBOSE=1 ft_vor

freertos_blinky:
	@make -C build freertos_blinky

demo:
	@make -C build demo

bootloader:
	@make -C build bootloader

can_bootloader:
	@make -C build can_bootloader

loader:
	@make -C build loader

eraser:
	@make -C build eraser

all:
	@make -C build all
