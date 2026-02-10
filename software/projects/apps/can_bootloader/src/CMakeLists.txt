set(src "CAN_BOOTLOADER_SRC")

file(GLOB FILES *.c startup_va416xx.s vectrst_gcc.s)
list(APPEND "${src}" "${FILES}")
set("${src}" "${${src}}" PARENT_SCOPE)
