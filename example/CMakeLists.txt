add_executable(oled_example main.cpp)

target_link_libraries(oled_example
    PRIVATE
        oled
        pthread
)

target_compile_features(oled_example PRIVATE cxx_std_20)

# Install (optional)
install(TARGETS oled_example RUNTIME DESTINATION bin)