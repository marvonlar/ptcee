# Copyright (c) 2022 Norwegian Defence Research Establishment (FFI)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

macro(configure_module libname modulename)
  set(CMAKE_DEBUG_POSTFIX "d")
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
  set_property(GLOBAL PROPERTY USE_FOLDERS ON)

  if (MSVC)
    set(compiler_options
      /W4
      /WX
      )
  else ()
    set(compiler_options
      -Wall
      -Wcast-align
      -Wcast-qual
      -Werror
      -Wextra
      -Wfloat-conversion
      -Winit-self
      -Winit-self
      -Wlogical-op
      -Wmissing-declarations
      -Wnon-virtual-dtor
      -Wold-style-cast
      -Woverloaded-virtual
      -Wpedantic
      -Wpointer-arith
      -Wshadow
      -Wsuggest-override
      -Wuninitialized
      -Wunknown-pragmas
      -Wunreachable-code
      -Wunused-local-typedefs
      )
  endif (MSVC)

  message(STATUS "* Adding module '${PROJECT_NAME}'")

  add_library(${libname}::${modulename} ALIAS ${PROJECT_NAME})

  target_compile_options(${PROJECT_NAME} PRIVATE ${compiler_options})

  target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    )

  set_target_properties(${PROJECT_NAME} PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    LINKER_LANGUAGE CXX
    CXX_STANDARD_REQUIRED ON
    CXX_STANDARD 17
    SOVERSION ${PROJECT_VERSION}
    BUILD_RPATH $ORIGIN
    INSTALL_RPATH $ORIGIN
    )

  # Lowercase all installed library files
  string(TOLOWER "${PROJECT_NAME}" output_name)
  set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME ${output_name})

  install(
    DIRECTORY "include/"
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING PATTERN "*.h"
    )

  install(TARGETS ${PROJECT_NAME}
    EXPORT ${targets_export_name}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )

  install(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/LICENSE
    DESTINATION "."
    )
endmacro()
