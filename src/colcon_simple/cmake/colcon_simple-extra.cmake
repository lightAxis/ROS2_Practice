if(_COLCON_SIMPLE_EXTRA_INCLUDED_)
    return()
endif()
set(_COLCON_SIMPLE_EXTRA_INCLUDED_ TRUE)

# colcon
macro(colcon_simple)
    if(NOT CMAKE_CXX_STANDARD)
        set(CMAKE_CXX_STANDARD 14)
    endif()
    if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
    endif()

    # Arguments
    # ALL_DEPS_REQUIRED                 -- Add the "REQUIRED" flag when calling find_package() for each dependancy
    # VERBOSE                           -- Display message of this macro
    # USE_INTERFACE_FROM_SAME_PACKAGE   -- Link extra typesupport for generated executables to use interface generated in same package
    # 
    cmake_parse_arguments(cs_args "ALL_DEPS_REQUIRED;VERBOSE;USE_INTERFACE_FROM_SAME_PACKAGE" "" "" ${ARGN})

    if(${cs_args_VERBOSE})
        message("[colcon_simple : ${PROJECT_NAME}] --colcon_simple(${ARGV})--")
    endif()

    if(TARGET {PROJECT_NAME}_package)
        message(WARNING "Could not create target '${${PROJECT_NAME}_package}' for project ${PROJECT_NAME}, as it already exists.")
    endif()
    add_custom_target(${PROJECT_NAME}_package)
    set(${PROJECT_NAME}_TARGETS)
    set(${PROJECT_NAME}_LIBRARIES)
    set(${PROJECT_NAME}_MSGS)
    set(${PROJECT_NAME}_SRVS)
    set(${PROJECT_NAME}_ACTIONS)
    set(${PROJECT_NAME}_LOCAL_MSG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/msg)
    set(${PROJECT_NAME}_LOCAL_SRV_DIR ${CMAKE_CURRENT_SOURCE_DIR}/srv)
    set(${PROJECT_NAME}_LOCAL_ACTION_DIR ${CMAKE_CURRENT_SOURCE_DIR}/action)
    set(${PROJECT_NAME}_LOCAL_PARAM_DIR ${CMAKE_CURRENT_SOURCE_DIR}/param)
    set(${PROJECT_NAME}_LOCAL_LAUNCH_DIR ${CMAKE_CURRENT_SOURCE_DIR}/launch)

    if(${cs_args_VERBOSE})
        message("[colcon_simple : ${PROJECT_NAME}] include dir : include")
    endif()
    include_directories(include)

    
    find_package(ament_cmake REQUIRED)
    if(NOT DEFINED _AMENT_PACKAGE_NAME)
        # _ament_package_xml(${CMAKE_CURRENT_BINARY_DIR}/ament_cmake_core ${ARGN})
        ament_package_xml()
    endif()

    # automatic package finding
    set(${PROJECT_NAME}_PKGS_TO_FIND)
    list(APPEND ${PROJECT_NAME}_PKGS_TO_FIND ${${_AMENT_PACKAGE_NAME}_BUILD_DEPENDS})
    list(APPEND ${PROJECT_NAME}_PKGS_TO_FIND ${${_AMENT_PACKAGE_NAME}_BUILDTOOL_DEPENDS})
    foreach(loopVar IN LISTS ${PROJECT_NAME}_PKGS_TO_FIND LISTS)
        # If the flat is defined, add the "REQUIRED" flag to all find_package()
        if(${cs_args_ALL_DEPS_REQUIRED})
            find_package(${loopVar} REQUIRED)
            if(${cs_args_VERBOSE})
                message("[colcon_simple : ${PROJECT_NAME}] find_package (REQUIRED) : ${loopVar}")
            endif()
        else()
            find_package(${loopVar} QUIET)
            if(${cs_args_VERBOSE})
                message("[colcon_simple : ${PROJECT_NAME}] find_package (QUIET) : ${loopVar}")
            endif()
        endif()
    endforeach()

    # check the param directory exists
    set(${PROJECT_NAME}_PARAM_EXIST false)
    if(IS_DIRECTORY ${${PROJECT_NAME}_LOCAL_PARAM_DIR})
        if({cs_args_VERBOSE})
            message("[colcon_simple : ${PROJECT_NAME}] found param dir")
        endif()
        set(${PROJECT_NAME}_PARAM_EXIST true)
    endif()

    # check the launch directory exists
    set(${PROJECT_NAME}_LAUNCH_EXIST false)
    if(IS_DIRECTORY ${${PROJECT_NAME}_LOCAL_LAUNCH_DIR})
        if({cs_args_VERBOSE})
            message("[colcon_simple : ${PROJECT_NAME}] found launch dir")
        endif()
        set(${PROJECT_NAME}_LAUNCH_EXIST true)
    endif()

    # automatic msg/arv/action finding prepare
    # check the msg directory exists & parse all msg files to list
    set(${PROJECT_NAME}_MSG_GENERATION false)
    if(IS_DIRECTORY ${${PROJECT_NAME}_LOCAL_MSG_DIR})
        file(GLOB_RECURSE ${PROJECT_NAME}_MSGS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} msg/*.msg)
        if(${PROJECT_NAME}_MSGS)
            if(${cs_args_VERBOSE})
                message("[colcon_simple : ${PROJECT_NAME}] found msgs")
            endif()
            set(${PROJECT_NAME}_MSG_GENERATION true)
        endif()
    endif()

    # check the srv directory exists & parse all srv files to list
    set(${PROJECT_NAME}_SRV_GENERATION false)
    if(IS_DIRECTORY ${${PROJECT_NAME}_LOCAL_SRV_DIR})
        file(GLOB_RECURSE ${PROJECT_NAME}_SRVS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} srv/*.srv)
        if(${PROJECT_NAME}_SRVS)
            if(${cs_args_VERBOSE})
                message("[colcon_simple : ${PROJECT_NAME}] found srvs")
            endif()
            set(${PROJECT_NAME}_SRV_GENERATION true)
        endif()
    endif()

    # check the action directory exists & parse all action files to list
    set(${PROJECT_NAME}_ACTION_GENERATION false)
    if(IS_DIRECTORY ${${PROJECT_NAME}_LOCAL_ACTION_DIR})
        file(GLOB_RECURSE ${PROJECT_NAME}_ACTIONS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} action/*.action)
        if(${PROJECT_NAME}_ACTIONS)
            if(${cs_args_VERBOSE})
                message("[colcon_simple : ${PROJECT_NAME}] found actions")
            endif()
            set(${PROJECT_NAME}_ACTION_GENERATION true)
        endif()
    endif()

    # is essential build dependencies are avialable in package.xml?
    set(${PROJECT_NAME}_IDL_GENERATION false)
    if(${${PROJECT_NAME}_MSG_GENERATION} OR ${${PROJECT_NAME}_SRV_GENERATION} OR ${${PROJECT_NAME}_ACTION_GENERATION})

        # is rosidl_default_genrators included?
        set(${PROJECT_NAME}_IDL_GENERATION_DEFAULT_GENERATORS_INCLUDED false)
        if("rosidl_default_generators" IN_LIST ${_AMENT_PACKAGE_NAME}_BUILDTOOL_DEPENDS)
            find_package(rosidl_default_generators REQUIRED)
            set(${PROJECT_NAME}_IDL_GENERATION_DEFAULT_GENERATORS_INCLUDED true)
        else()
            message(WARNING "[colcon_simple : ${PROJECT_NAME}] Need <buildtool_depend>rosidl_default_generators</buildtool_depend> in package.xml!")
        endif()

        # is rosidl_default_runtime included?
        set(${PROJECT_NAME}_IDL_GENERATION_DEFAULT_RUNTIME_INCLUDED false)
        if("rosidl_default_runtime" IN_LIST ${_AMENT_PACKAGE_NAME}_EXEC_DEPENDS)
            find_package(rosidl_default_runtime REQUIRED)
            set(${PROJECT_NAME}_IDL_GENERATION_DEFAULT_RUNTIME_INCLUDED true)
        else()
            message(WARNING "[colcon_simple : ${PROJECT_NAME}] Needs <exec_depend>rosidl_default_runtime</exec_depend> in package.xml!")
        endif()

        # is rosidl_interface_packages included?
        set(${PROJECT_NAME}_IDL_GENERATION_INTERFACE_PACKAGES_INCLUDED false)
        if("rosidl_interface_packages" IN_LIST ${_AMENT_PACKAGE_NAME}_MEMBER_OF_GROUPS)
            
        else()
            message(WARNING "[colcon_simple : ${PROJECT_NAME}] Needs <member_of_group>rosidl_interface_packages</member_of_group> in package.xml!")
        endif()

        # is rosidl generation is available?
        if(${PROJECT_NAME}_IDL_GENERATION_DEFAULT_GENERATORS_INCLUDED AND ${PROJECT_NAME}_IDL_GENERATION_DEFAULT_RUNTIME_INCLUDED)
        set(${PROJECT_NAME}_IDL_GENERATION true)
        endif()
       
    endif()

    # start idl generation only when idl generation dependencies are available
    if(${PROJECT_NAME}_IDL_GENERATION)

        set(${PROJECT_NAME}_IDL_TARGETS)
        if(${${PROJECT_NAME}_MSG_GENERATION})
            if(${cs_args_VERBOSE})
                foreach(loopVar IN LISTS ${PROJECT_NAME}_MSGS)
                    message("[colcon_simple : ${PROJECT_NAME}] add msg : ${loopVar}")
                endforeach()
            endif()
            list(APPEND ${PROJECT_NAME}_IDL_TARGETS ${${PROJECT_NAME}_MSGS})
        endif()

        if(${${PROJECT_NAME}_SRV_GENERATION})
            if(${cs_args_VERBOSE})
                foreach(loopVar IN LISTS ${PROJECT_NAME}_SRVS)
                    message("[colcon_simple : ${PROJECT_NAME}] add srv : ${loopVar}")
                endforeach()
            endif()
            list(APPEND ${PROJECT_NAME}_IDL_TARGETS ${${PROJECT_NAME}_SRVS})
        endif()

        if(${${PROJECT_NAME}_ACTION_GENERATION})
            if(${cs_args_VERBOSE})
                foreach(loopVar IN LISTS ${PROJECT_NAME}_ACTIONS)
                    message("[colcon_simple : ${PROJECT_NAME}] add action : ${loopVar}")
                endforeach()
            endif()
            list(APPEND ${PROJECT_NAME}_IDL_TARGETS ${${PROJECT_NAME}_ACTIONS})
        endif()

         # generate rosidl from idl targets and dependencies from package.xml
        if(${cs_args_VERBOSE})
            message("[colcon_simple : ${PROJECT_NAME}] generate rosidl interfaces with dependencies : ${${_AMENT_PACKAGE_NAME}_BUILD_DEPENDS}")
        endif()
        rosidl_generate_interfaces(${PROJECT_NAME}
            ${${PROJECT_NAME}_IDL_TARGETS}
            DEPENDENCIES ${${_AMENT_PACKAGE_NAME}_BUILD_DEPENDS})
        
        # if use interface from same package is on, make cpp_typesupport_target
        set(${PROJECT_NAME}_USE_INTERFACE_FROM_SAME_PACKAGE false)
        if(${cs_args_USE_INTERFACE_FROM_SAME_PACKAGE})
            if(${cs_args_VERBOSE})
                message("[colcon_simple : ${PROJECT_NAME}] add cpp_typesupport_target as USE_INTERFACE_FROM_PACKAGE applied")
            endif()
            rosidl_get_typesupport_target(cpp_typesupport_target
                ${PROJECT_NAME} rosidl_typesupport_cpp)
            set(${PROJECT_NAME}_USE_INTERFACE_FROM_SAME_PACKAGE true)
        endif()

    endif()
endmacro(colcon_simple)



macro(cs_add_executable)
    # Arguments
    # VERBOSE -- display message of this macro
    cmake_parse_arguments(cs_args "VERBOSE" "" "" ${ARGV})

    if(${cs_args_VERBOSE})
        message("[colcon_simple : ${PROJECT_NAME}] --cs_add_executable(${ARGV})--")
    endif()

    set(cs_args_TARGET_ARGV)
    foreach(loopVar IN ITEMS ${ARGV})
        if(NOT ${loopVar} STREQUAL "VERBOSE")
            list(APPEND cs_args_TARGET_ARGV ${loopVar})
        endif()
    endforeach()

    list(GET cs_args_TARGET_ARGV 0 cs_args_TARGET_NAME)
    if(${cs_args_VERBOSE})
        message("[colcon_simple : ${PROJECT_NAME}] add executable : ${cs_args_TARGET_NAME}")
    endif()

    add_executable(${cs_args_TARGET_ARGV})
    ament_target_dependencies(${cs_args_TARGET_NAME} ${${_AMENT_PACKAGE_NAME}_BUILD_DEPENDS})
    if(${cs_args_VERBOSE})
        message("[colcon_simple : ${PROJECT_NAME}] ament target dependencies : [${cs_args_TARGET_NAME}] <- ${${_AMENT_PACKAGE_NAME}_BUILD_DEPENDS}")
    endif()
    if(${PROJECT_NAME}_USE_INTERFACE_FROM_SAME_PACKAGE)
        if(${cs_args_VERBOSE})
            message("[colcon_simple : ${PROJECT_NAME}] Use interface from same package : ON, target link libraries, ${cpp_typesupport_target}")
        endif()
        target_link_libraries(${cs_args_TARGET_NAME} "${cpp_typesupport_target}")
    endif()
    list(APPEND ${PROJECT_NAME}_TARGETS ${cs_args_TARGET_NAME})

endmacro(cs_add_executable)


macro(cs_install)
    # Arguments
    # VERBOSE -- display message of this macro
    cmake_parse_arguments(cs_args "VERBOSE" "" "" ${ARGN})

    if(${cs_args_VERBOSE})
        message("[colcon_simple : ${PROJECT_NAME}] --cs_install(${ARGV})--")
    endif()

    if(${cs_args_VERBOSE})
        foreach(loopVar IN LISTS ${PROJECT_NAME}_TARGETS)
            message("[colcon_simple : ${PROJECT_NAME}] installing exec: ${loopVar} -> lib/${PROJECT_NAME}")
        endforeach()
    endif()

    # install all targets generated to lib dir
    install(TARGETS ${${PROJECT_NAME}_TARGETS}
        DESTINATION lib/${PROJECT_NAME})

    # install param dir if exist
    if(${PROJECT_NAME}_PARAM_EXIST)
        if(${cs_args_VERBOSE})
            message("[colcon_simple : ${PROJECT_NAME}] installing dir : param -> share/${PROJECT_NAME}")
        endif()
        install(DIRECTORY param
            DESTINATION share/${PROJECT_NAME})
    endif()

    # install launch dir if exist
    if(${PROJECT_NAME}_LAUNCH_EXIST)
        if(${cs_args_VERBOSE})
            message("[colcon_simple : ${PROJECT_NAME}] installing dir : launch -> share/${PROJECT_NAME}")
        endif()
        install(DIRECTORY launch
            DESTINATION share/${PROJECT_NAME})
    endif()
    

endmacro()

macro(cs_end)
    # Arguments
    # VERBOSE -- display message of this macro
    cmake_parse_arguments(cs_args "VERBOSE" "" "" ${ARGN})

    if(${cs_args_VERBOSE})
        message("[colcon_simple : ${PROJECT_NAME}] --cs_end(${ARGV})--")
    endif()

    if(${PROJECT_NAME}_IDL_GENERATION)
        if(${cs_args_VERBOSE})
            message("[colcon_simple : ${PROJECT_NAME}] IDL Generation detected, adding : ament_exeport_dependencies(rosidl_default_runtime)")
        endif()
        ament_export_dependencies(rosidl_default_runtime)
    endif()

    if(${cs_args_VERBOSE})
        message("[colcon_simple : ${PROJECT_NAME}] finished, adding : ament_package()")
    endif()
    ament_package()
endmacro()