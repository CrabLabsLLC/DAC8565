default:
    @just --list

# Format every C source/header in place
format:
    find . -regex '.*\.\(c\|h\)' -not -path './build/*' -exec clang-format -i {} \;

# Check formatting without writing changes; non-zero exit on any diff
format_check:
    find . -regex '.*\.\(c\|h\)' -not -path './build/*' -exec clang-format --dry-run --Werror {} +

# Build the standalone CMake library
build:
    mkdir -p build
    cd build && cmake .. -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
    ln -sf build/compile_commands.json compile_commands.json
    cd build && make

clean:
    rm -rf build compile_commands.json
