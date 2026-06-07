# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(bap_unicast_client_4)

target_sources(app PRIVATE
  src/main_3.c
  src/app_state.c
  src/stream_pdm.c
)

zephyr_sources_ifdef(CONFIG_LIBLC3 src/stream_lc3.c)
zephyr_sources_ifdef(CONFIG_BT_AUDIO_TX src/stream_tx.c)

zephyr_library_include_directories(app PRIVATE src)
