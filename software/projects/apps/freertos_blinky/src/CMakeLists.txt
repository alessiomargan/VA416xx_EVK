set(src "FREERTOS_BLINKY_SRC")

file(GLOB FILES *.c startup_va416xx.s)
list(APPEND "${src}" "${FILES}")
set("${src}" "${${src}}" PARENT_SCOPE)
