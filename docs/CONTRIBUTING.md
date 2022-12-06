# Contributing to SPEAR

The SPEAR team is glad to accept contributions from anybody willing to collaborate. There are different ways to contribute to the project, depending on the capabilities of the contributor. The team will work as much as possible so that contributions are successfully integrated in SPEAR.

## Report Bugs

Issues can be reported in the [issue](https://github.com/isl-org/spear/issues) section on GitHub. Before reporting a new bug, make sure to do some checkups.

- Check if the bug has been reported. Look it up in that same issue section on GitHub.

- Read the docs. Make sure that the issue is a bug, and not a misunderstanding on how SPEAR is supposed to work.

## Request Features

Ideas for new features are also a great way to contribute. Any suggestion that could improve the users' experience can be submitted in the corresponding GitHub section [here](https://github.com/isl-org/spear/issues).

## Code Contributions

In order to start working, fork the SPEAR repository, and clone the fork in your computer. Remember to keep your fork in sync with the original repository.

Please follow the current [coding guidelines](https://github.com/isl-org/spear/blob/main/docs/coding_guidelines.md) when submitting new code.

## Submitting a new PR

Contributions and new features are not merged directly to the `main` branch, but to an intermediate branch named `dev`. This Gitflow branching model makes it easier to maintain a stable `main` branch. This model requires a specific workflow for contributions.

- Always keep your branch updated with the lastest changes from `dev` branch.
- Develop the contribution in child branch named as `username/name_of_the_contribution`.
- Once the contribution is ready, submit a pull-request from your branch to `dev`. Try to be as descriptive as possible when filling the description.
- Once the contribution is merged to `dev`, it can be tested with the rest of new features. By the time of the next release, the `dev` branch will be merged to `main`, and the contribution will be available and announced.
