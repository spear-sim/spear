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

- The [script](utils/apply_clang_style.sh) applies this style to all C++ files in this repository.

### Python

- We use `black` for python formatting. You can install it with `pip install black`. (It is already contained in the python [requirements file](requirements.txt).)

- To format all python files, run `black .` from the root folder of this repository.
