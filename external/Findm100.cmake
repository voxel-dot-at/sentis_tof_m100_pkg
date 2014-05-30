# - Try to find PMD headers and plugins
# Once done this will define
#
FIND_PATH(m100_INCLUDE_DIR NAMES m100api.h
 	PATHS
  		/usr/include/libm100/
		/usr/local/include/libm100/
		../library/include/
 	NO_DEFAULT_PATH
	DOC "Include directory of m100api"
)

	if (CMAKE_CL_64)
      set(ARCH_DIR "build64")
  	else()
      set(ARCH_DIR "build")
  	endif()
find_library(m100_LIBRARY NAMES m100
        PATHS
  		/usr/lib/
		/usr/local/lib/
		../${ARCH_DIR}/Debug/
	NO_DEFAULT_PATH
	DOC "Library binary"
)

#Not needed by now as we have just 1 include and lib
#set(m100_LIBRARIES ${m100_LIBRARY} )
#set(m100_INCLUDE_DIRS ${m100_INCLUDE_DIR} )

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set M100_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(m100  DEFAULT_MSG
                                  m100_LIBRARY m100_INCLUDE_DIR)

mark_as_advanced(m100_INCLUDE_DIR m100_LIBRARY)


