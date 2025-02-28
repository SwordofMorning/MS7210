#!/bin/bash

#################### Section 1 : Echo ####################

RED='\033[1;31m'
CYAN='\033[1;36m'
GREEN='\033[1;32m'
NC='\033[0m'

function echo_error()
{
    local string="$1"
    echo -e "${CYAN}${ScriptEchoLabel}${RED}${1}${NC}"
}

function echo_info()
{
    local string="$1"
    echo -e "${CYAN}${ScriptEchoLabel}${GREEN}${1}${NC}"
}

#################### Section 2 : Project Setting ####################

# Project Default Parameters
BuildPath="build"

ProjectPath=$(cd $(dirname $0);pwd)
ScriptEchoLabel="Build.sh: "

#################### Section 3 : Functions ####################

# if no BuildPath, then Create
function func_dir()
{
    cd ${ProjectPath}
    if [ ! -d "$BuildPath" ]; then
        mkdir -p "$BuildPath"
        echo_info "Create Build Path: $BuildPath"
    fi
}

# camke ..
function func_cmake()
{
    echo_info "cmake .."
    if cmake ..; then
        echo_info "CMake Success."
    else
        echo_error "CMake Fail."
        exit 1
    fi
}

# make
function func_make() 
{
    echo_info "make -j12"
    if make -j12; then
        echo_info "Make Success."
    else 
        echo_error "Make Fail."
        exit 1
    fi
}

function func_all
{
    cd ${BuildPath}
    func_cmake
    func_make
}

# clean
function func_clean()
{
    echo_info "clean"
    if [[ -d "${ProjectPath}/${BuildPath}" ]]; then
        rm -r "${ProjectPath}/${BuildPath}"
        if [[ $? -eq 0 ]]; then
            echo_info "Clean Success."
        else
            echo_error "Clean Fail."
            exit 1
        fi
    else
        echo_info "Directory ${ProjectPath}/${BuildPath} does not exist."
    fi
}

# help
function func_help()
{
    echo_info "(no parameters)          -- create output path, cmake, make"
    echo_info "[all]                    -- clean, create output path, camke, make"
    echo_info "[clean]                  -- rm output path"
    echo_info "[help]                   -- show helps"
}

function func_map()
{
    if [[ $1 == 'all' ]]; then
        func_clean
        func_dir
        func_all
    elif [[ $1 == "clean" ]]; then
        func_clean
    elif [[ $1 == "help" ]]; then
        func_help
    elif [[ -z $1 ]]; then
        func_dir
        func_all
    else
        func_help
    fi
}

function cf()
{
    cd ${ProjectPath}
    find src others -name '*.cpp' -o -name '*.h' -o -name '*.c' | \
        xargs -I{} bash -c 'clang-format-16 -style=file {} | diff -u -L "{}" -L "{}" {} -' > .clang-format.diff
}

#################### Section 4 : Main ####################

function main()
{
    clear

    echo_info "PWD: ${ProjectPath}"
    echo_info "Output: ${ProjectPath}/${BuildPath}"

    func_map ${1}
    cf
}

main $1