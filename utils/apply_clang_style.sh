#!/bin/sh

find .. \( -name '*.h' -or -name '*.cpp' \) | xargs clang-format -i -style=file --verbose
