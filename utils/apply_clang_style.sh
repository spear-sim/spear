#!/bin/sh

find ../code/ \( -name '*.h' -or -name '*.cpp' \) -not -ipath '*/ThirdParty/*' | xargs clang-format -i -style=file --verbose
