# SPDX-License-Identifier: Apache-2.0 OR MIT

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(Project)

file(GLOB SRC_C_FILES CONFIGURE_DEPENDS "src/*.c")
target_sources(app PRIVATE ${SRC_C_FILES})

rust_cargo_application()

