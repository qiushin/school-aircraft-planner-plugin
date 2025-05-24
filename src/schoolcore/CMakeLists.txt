set(CMAKE_AUTOUIC ON)

file(GLOB CORE_SOURCES
    "*.cpp"
)

file(GLOB CORE_HEADERS
    "*.h"
)

set(UI_FILES
    MainWindow.ui
)

add_library(schoolcore STATIC
    ${CORE_SOURCES}
    ${CORE_HEADERS}
    ${UI_FILES}
)

target_include_directories(schoolcore PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${INCLUDES}
)

target_link_libraries(schoolcore
    ${LIBS}
)

