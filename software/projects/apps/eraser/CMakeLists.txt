set(src "ERASER_SRC")

file(GLOB FILES *.c *.s)
list(APPEND "${src}" "${FILES}")
set("${src}" "${${src}}" PARENT_SCOPE)
