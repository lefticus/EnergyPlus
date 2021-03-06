project(HVAC-Diagram Fortran)

set( CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/../../Products" )
set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/../../Products" )
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/../../Products" )

SET(SRC HVAC-Diagram-Main.f90) 

if(APPLE)
  if(CMAKE_Fortran_COMPILER MATCHES "ifort")
    set(CMAKE_Fortran_FLAGS "${CMAKE_Fortran_FLAGS} -static-intel" )
  else()
    set(CMAKE_Fortran_FLAGS "${CMAKE_Fortran_FLAGS} -std=legacy"  )
    set(CMAKE_Fortran_FLAGS "${CMAKE_Fortran_FLAGS} -ffree-line-length-200" )
  endif()
elseif(UNIX)
  set(CMAKE_SKIP_RPATH TRUE)
  if(CMAKE_Fortran_COMPILER MATCHES "ifort")
    set(CMAKE_Fortran_FLAGS "${CMAKE_Fortran_FLAGS} -static-intel" )
  else()
    set(CMAKE_Fortran_FLAGS "${CMAKE_Fortran_FLAGS} -static" )
    set(CMAKE_Fortran_FLAGS "${CMAKE_Fortran_FLAGS} -std=legacy"  )
    set(CMAKE_Fortran_FLAGS "${CMAKE_Fortran_FLAGS} -ffree-line-length-200" )
  endif()
else()
  if(CMAKE_Fortran_COMPILER MATCHES "ifort")
    # Set release flags to be empty
    set(CMAKE_Fortran_FLAGS_RELEASE "")
    set(CMAKE_Fortran_FLAGS "${CMAKE_Fortran_FLAGS} /libs:static")
  else()
    set(CMAKE_Fortran_FLAGS "${CMAKE_Fortran_FLAGS} -static")
    set(CMAKE_Fortran_FLAGS "${CMAKE_Fortran_FLAGS} -std=legacy"  )
    set(CMAKE_Fortran_FLAGS "${CMAKE_Fortran_FLAGS} -ffree-line-length-200" )
  endif()
endif()

set(SKIP_BUILD_RPATH true)
ADD_EXECUTABLE( HVAC-Diagram ${SRC} )

include("${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/ProjectMacros.cmake")
install_and_fixup_exe_target( HVAC-Diagram PostProcess )

