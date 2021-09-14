#!/bin/sh

find .. \( -name '*.h' -or -name '*.cpp' \) | xargs clang-format-8 -i -style=file --verbose
