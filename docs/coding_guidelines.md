# Coding Guidelines

We generally adhere to the [AirSim Coding Guidelines](https://microsoft.github.io/AirSim/coding_guidelines/) except as noted below. In cases where the AirSim guidelines don't make a recommendation, we generally agree with the recommendations in the [ISO C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines) except as noted below. The AirSim guidelines are mostly a style guide (e.g., how to name variables), whereas the ISO C++ guidelines give more substantial recommendations on how and when to use language features (e.g., how to make effective use of move semantics).

## Our design philosophy

- Prefer code that is as small and simple as possible
- Only implement functionality that we know we need
- Be as consistent as possible with everything (e.g., data flow, interface design, naming conventions, documentation, etc)
- Avoid trivial pass-through layers
- Avoid excessive folder hierarchy
- System behavior should be as symmetric as possible (e.g., if there is an `initialize()` method, there should also be a `terminate()` method that returns the system into the state it was in before being initialized)
- System behavior should be as stateless as possible
- Interfaces should be as narrow as possible
- Classes, functions, and variables should be as private and as local as possible
- If system behavior only needs to be set once during startup, then use our config system rather than creating `get()`/`set()` methods throughout the code
- Integrate code into the `main` branch as frequently as possible (e.g., in one-week intervals or less)
- Clean and re-factor as you go

## Our guidelines

The guidelines below may disagree with the AirSim and ISO C++ guidelines, but we adhere to them anyway.

### General

**Use asserts instead of throwing exceptions.** We want to catch all errors as soon as they happen, without giving surrounding code an opportunity to continue executing. In C++, always use our `ASSERT` macro. Don't use Unreal Engine's `check` macro, because it is hard to enable `check` in Shipping builds.

```python
# bad
if world is None
    throw Exception()

# good
assert world is not None
```

```cpp
// bad
if (!world) {
    throw std::exception();
}

// bad 
check(world);

// good
ASSERT(world);
```

**Use your own human judgement rather than `clang-format` and `black` to format your code.** `clang-format` is not idempotent, can get stuck in cycles, and requires a lot of configuration to behave sensibly. Likewise, `black` requires a lot of configuration to behave sensibly.

**Delete merged branches, and prefer "squash and merge" over "merge" when merging pull requests.** We need to delete branches as they're merged to prevent them from accumulating. We prefer "squash and merge" so we don't clutter our `main` branch with bad commit messages.

### Python

**Use the following naming conventions.** We need a naming convention for our Python code, and this one roughly agrees with the rest of the scientific Python stack (e.g., NumPy, SciPy, matplotlib, Pandas, PyTorch, etc).

- `leading_lowercase_underscore` for variables, functions, methods, modules, and filenames
- `LeadingUppercaseCamelCase` for classes
- `self._leading_underscore` for member variables and methods that are intended to be private
- Spaces instead of tabs with four spaces per indent

**Don't use type annotations.** It's easy to forget to update these annotations as the code changes, in which case they become actively misleading. In some cases, an object you get from a third-party library can be one of several types, leading to an annotation that is either wrong (and therefore misleading) or excessively verbose. These annotations also interfere with readability and expressiveness, and this project is too small to benefit significantly from their use.

```python
# bad
def add(x: Union[int, List[int], float, List[float], y: Union[int, List[int], float, List[float]]) -> Union[int, float]:
    if isinstance(x, list):
        x_: Union[int, float] = reduce(sum, x)
    if isinstance(y, list):
        y_: Union[int, float] = reduce(sum, y)        
    sum: Union[int, float] = x_ + y_
    return sum

# good
def add(x, y):
    if isinstance(x, list):
        x = reduce(sum, x)
    if isinstance(y, list):
        y = reduce(sum, y)        
    sum = x + y
    return sum
```

**Don't use docstrings (for now).** Docstrings are similar to public-facing README documents. They are useful when an interface has completely stabilized, and that interface needs to be communicated to novice users. Neither of these conditions currently applies to this project, and docstrings are therefore not worth the maintenance effort. This may change in future as our Python interfaces stabilize and we get closer to a public release.

```python
# bad: too much maintenance effort

class Photo(ndarray):
    """
    Array with associated photographic information.

    Attributes
    ----------
    exposure : float
        Exposure in seconds.

    Methods
    -------
    colorspace(c='rgb')
        Represent the photo in the given colorspace.
    gamma(n=1.0)
        Change the photo's gamma exposure.
    """

# good: minimal maintenance effort

# array with associated photographic information
class Photo(ndarray):
```

### C++

**Use braces even for one-line if statements.** It's easy to forget to add braces when adding code to the body of the if statement.

```cpp
// bad
if (x < 0)
  break;
else
  break;
  
// good
if (x < 0) {
  break;
} else {
  break;
}
```

**Use `#pragma once`.** It's easy to forget to rename `PATH_TO_FILE_H` if you rename or move `file.h`

```cpp
// bad
#ifndef PATH_TO_FILE_H
#define PATH_TO_FILE_H
// ...
#endif

// good
#pragma once
// ...
```

**Don't use `/* */` comments.** They can't be nested recursively, whereas `//` comments can be nested recursively. Therefore `//` comments make it easier to comment and uncomment big chunks of code.

```cpp
/* bad */
i += 1;

// good
i += 1;
```

**Group headers logically.** Group headers belonging to each library together, put low-level fundamental libraries (e.g., the standard library) closer to the top of the file, arrange headers alphabetically within each group, reserve `<>` for third-party headers, and use `""` for our headers. It makes the code easier to read and understand. It is helpful to understand at a glance what headers are part of our code, and what headers belong to third-party libraries.

```cpp
// bad
#include <MyHeader.h>
#include <vector>
#include "map"
#include "UnrealEngine/UnrealHeader.hpp"
#include <UnrealEngine/AnotherUnrealHeader.hpp>

// good
#include <map>
#include <vector>

#include <UnrealEngine/AnotherUnrealHeader.hpp>
#include <UnrealEngine/UnrealHeader.hpp>

#include "MyHeader.h"
```

**Prefer `std` types (e.g., `std::string`, `std::vector`) over Unreal types (e.g., `FString`, `TArray`).** It is ok to use Unreal types, but there should be a good reason for doing so (e.g., you need to interact with an Unreal function). Using `std` types make it easier for new developers to understand the code and to contribute.

```cpp
// bad
FString my_string = "My String";

// good
std::string = "My string";
```

**Prefer `std::unique_ptr` over a raw pointer, when an object owns a pointer to another object.** It is easy to accidentally forget to delete raw pointers.

```cpp
// bad
Object* my_object = new Object(123.0f);

// good
std::unique_ptr<Object> my_object = std::make_unique<Object>(123.0f);
```

**Always use `at()` to get an existing item from an `std::map`.** Using `[]` to get an existing item will silently create a default item if it doesn't already exist, which is not desirable behavior. For consistency, always use `[]` to insert a new item.

```cpp
// bad
int index = names_to_indices["name"];

// good
int index = names_to_indices.at("name");
```

**Headers should not contain implementations.** The only exceptions to this rule are templated classes and functions, and header files that contain only trivial single-line functions (i.e., where putting the implementations in the header avoids the need for a source file entirely). In the case of templated classes and functions, the header must contain declarations at the top of the file, and then definitions after.
