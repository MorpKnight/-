# Gather all .cpp files from the current directory (.)
file(GLOB SRCS "*.cpp")

# Include the current directory for header files
idf_component_register(SRCS ${SRCS}  # Automatically include all .cpp files
                       INCLUDE_DIRS "." "../arduino/cores/esp32"  # Include headers
                       REQUIRES arduino)  # If you need Arduino core