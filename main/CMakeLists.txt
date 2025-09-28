idf_component_register(
    SRCS "hf_mmc.c"
    INCLUDE_DIRS "."
    REQUIRES algo   # depend on algo component
)
