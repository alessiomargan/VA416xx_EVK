set(src "TESTS_SRC")

file(GLOB FILES *.c)
list(APPEND "${src}" "${FILES}")
set("${src}" "${${src}}" PARENT_SCOPE)
