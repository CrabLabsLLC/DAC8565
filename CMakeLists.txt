cmake_minimum_required(VERSION 3.20)

# DAC8565 driver — tri-mode build (ESP-IDF, Zephyr, plain CMake).
# See driver-guide.md §"Step 1: File Structure" for the rationale.

if(IDF_VERSION_MAJOR)
	# ── ESP-IDF Component ────────────────────────────────────────────
	idf_component_register(
		SRCS "src/dac8565.c"
		INCLUDE_DIRS "include"
	)
elseif(ZEPHYR_BASE)
	# ── Zephyr Module ────────────────────────────────────────────────
	zephyr_library()
	zephyr_library_sources(src/dac8565.c)
	zephyr_include_directories(include)
else()
	# ── Standard CMake (STM32, desktop tests, etc.) ──────────────────
	if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
		project(dac8565
			LANGUAGES C
			VERSION 0.1.0
			DESCRIPTION "Platform-agnostic driver for the TI DAC8565 quad-channel 16-bit DAC"
		)
	endif()

	add_library(dac8565 STATIC
		src/dac8565.c
	)
	target_include_directories(dac8565 PUBLIC include)
	target_compile_features(dac8565 PUBLIC c_std_11)
	target_compile_options(dac8565 PRIVATE -Wall -Wextra -Wpedantic)
endif()
