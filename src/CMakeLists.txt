
idf_component_register(
    SRCS "main.cpp"
    REQUIRES ulp bt
)

ulp_embed_binary(ulp_main "./ulp/ulp.c" "main.cpp")
