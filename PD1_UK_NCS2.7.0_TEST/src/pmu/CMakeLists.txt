#
# Copyright (c) 2019 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
#

zephyr_include_directories(.)
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/pmu.c)
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/npm1300.c)
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/max20353.c)
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/max20353_reg.c)
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/cw2215.c)
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/cw6307.c)
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/cw_interface.c)