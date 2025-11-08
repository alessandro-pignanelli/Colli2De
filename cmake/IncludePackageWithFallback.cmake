# Function to include packages with fallback to FetchContent
# Parameters:
#   PACKAGE_NAME - Name of the package (e.g., tsl-robin-map)
#   LIB_DIR - Directory name in libs/ folder (e.g., robin-map)
#   GIT_REPO - Git repository URL
#   VERSION - Required version (e.g., 1.4.0)
#   USE_LOCAL_VAR - Name of the CMake variable to force local (e.g., USE_LOCAL_ROBIN_MAP)
function(include_package_with_fallback PACKAGE_NAME LIB_DIR GIT_REPO VERSION USE_LOCAL_VAR)
    # Parse optional extra arguments as key-value pairs:
    # Example call:
    # include_package_with_fallback(... USE_LOCAL_VAR
    #     BUILD_TESTS OFF
    #     ANOTHER_OPTION ON
    # )
    set(extra_args ${ARGN})

    # Apply overrides before MakeAvailable
    while(extra_args)
        list(POP_FRONT extra_args key)
        list(POP_FRONT extra_args value)
        message(STATUS "Setting ${key}=${value} for ${PACKAGE_NAME}")
        set(${key} ${value} CACHE BOOL "" FORCE)
    endwhile()

    if (${USE_LOCAL_VAR})
        find_package(${PACKAGE_NAME} ${VERSION} REQUIRED PATHS ${COLLI2DE_SOURCE_DIR}/libs/${LIB_DIR}/install/${CMAKE_BUILD_TYPE})
    else()
        find_package(${PACKAGE_NAME} ${VERSION} QUIET PATHS ${COLLI2DE_SOURCE_DIR}/libs/${LIB_DIR}/install/${CMAKE_BUILD_TYPE})
        if (NOT ${PACKAGE_NAME}_FOUND)
            message(STATUS "${PACKAGE_NAME} not found, fetching from GitHub")
            include(FetchContent)
            FetchContent_Declare(
                ${PACKAGE_NAME}
                GIT_REPOSITORY ${GIT_REPO}
                GIT_TAG        v${VERSION}
            )
            FetchContent_MakeAvailable(${PACKAGE_NAME})
        else()
            message(STATUS "Using local ${PACKAGE_NAME} from ${${PACKAGE_NAME}_DIR}")
            set(${USE_LOCAL_VAR} ON CACHE BOOL "Use local ${PACKAGE_NAME}" FORCE)
        endif()
    endif()
endfunction()
