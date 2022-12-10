# Contribution Guidelines

## Submitting issues, questions, and feature requests

Issues with existing functionality, questions, and feature requests can be submitted [here](http://github.com/isl-org/spear/issues). Before submitting a new issue, search through the existing open and closed issues to see if your issue has already been reported.

## Fixing bugs and adding features

Before making modifications to our code, familiarize yourself with our [coding guidelines](docs/coding_guidelines.md). Once you are familiar with our guidelines, our process for fixing bugs and adding features is as follows.

1. Submit an issue describing the changes you want to implement. If it's a minor change or bug fix, you can skip to step 3.

2. After the scope is discussed in the issue, assign it to yourself.

3. Fork our repository and clone it locally.

```console
git clone https://github.com/<user-id>/spear.git
```

4. Create a branch with a concise and meaningful name that describes the scope of the work.

```console
git checkout -b <branch-name>
```

5. Make your changes, write good commit messages, push your branch to the forked repository:

```console
git add <modified file>
git commit -m <meaningful description>
git push --set-upstream origin <branch-name>
```

6. Make sure your branch is fully up-to-date with all of the latest changes in our `main` branch.

7. Create a pull request in the GitHub web interface and link the issue you submitted in step 1 to the pull request.

8. Address all feedback you receive and push it to your fork. The pull request will get updated automatically.

9. We will merge your pull request into our `main` branch when it is ready.
