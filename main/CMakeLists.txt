idf_component_register(SRCS "main.c" INCLUDE_DIRS "." EMBED_TXTFILES "index.html")
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-format-truncation)
