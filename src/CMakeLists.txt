rock_library(underwater_localization
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(underwater_localization_bin Main.cpp
    DEPS underwater_localization)

