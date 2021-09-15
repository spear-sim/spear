# InteriorSim

## Development

1) Create a new feature branch.

2) Raise a pull request to the `main` branch. Merging requires at least 1 approving review, and all GitHub actions need to pass.

## Python Requirements

The file [requirements.txt](requirements.txt) contains all python requirements. Please edit this file carefully, when an additional package is needed.

## Code Style

We require all files to be formatted in the defined formatting style before we allow a PR to be merged to the `main` branch.
This is realized via a GitHub [action](.github/workflows/check_style.yml).

### C++

- We use `clang-format`. See [.clang-format](.clang-format) for more details.

- The script [apply_clang_style.sh](utils/apply_clang_style.sh) applies this style to all C++ files in this repository. (This requires clang-format to be installed on your system; see below.)

- Since different versions of `clang-format` provide different results, we decided to use Version 10. (This also aligns with the clang version required for building UE 4.26 projects in Linux; see [this list](https://docs.unrealengine.com/4.27/en-US/SharingAndReleasing/Linux/GettingStarted/).)
Version 10 can be installed in the following way:
  - Windows: Download the [pre-build binaries](https://releases.llvm.org/download.html) (LLVM 10.0.0) and allow the option to add the clang toolchains to the PATH during installation.
  - MAC: Download the [pre-build binaries](https://releases.llvm.org/download.html) (LLVM 10.0.0) and add the bin folder to your PATH.
  - Linux: `sudo apt-get install clang-format-10`

- You can test if the correct version is installed with: `clang-format --version`

### Python

- We use `black` for python formatting. You can install it with `pip install black`. (It is already contained in the python [requirements file](requirements.txt).)

- To format all python files, run `black .` from the root folder of this repository.
