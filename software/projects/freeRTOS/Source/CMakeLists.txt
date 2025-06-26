set(src "FREERTOS_SRC")

file(GLOB FILES *.c portable/GCC/ARM_CM4F/*.c portable/MemMang/*.c)
# file(GLOB FILES *.c)
list(APPEND "${src}" "${FILES}")
set("${src}" "${${src}}" PARENT_SCOPE)
