# List all your source files
set(SOURCES
    "src/main.cpp"
    "src/ads129x_driver.cpp"
)

# Register the main component
idf_component_register(
    SRCS "${SOURCES}"           # Source files
    INCLUDE_DIRS "include"      # Public header directory
    PRIV_INCLUDE_DIRS "src"     # Private headers in src (optional but recommended)
)