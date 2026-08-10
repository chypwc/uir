These are three common files you'll encounter when compiling C++ on Linux:

```text
.cpp
 ↓ compile
.o          object file
 ↓
 ├─ archive → .a     static library
 │
 └─ link    → .so    shared library
```

They represent different stages/forms of compiled machine code.

## 1. `.o` — object file

`.o` means **object file**.

Suppose you have:

```text
rotation3.cpp
```

The compiler compiles it:

```bash
g++ -c rotation3.cpp
```

and produces:

```text
rotation3.o
```

The `.o` file contains **compiled machine code**, but it is generally not yet a complete executable or library.

For example:

```cpp
// rotation3.cpp

Eigen::Vector3d Rotation3::rotate_vector(...) const
{
    return matrix_ * vector_b;
}
```

roughly goes through:

```text
C++ source
rotation3.cpp
      ↓
   compiler
      ↓
machine instructions
rotation3.o
```

### Why not immediately create an executable?

Consider a larger project:

```text
src/
├── rotation3.cpp
├── transform3.cpp
├── quaternion.cpp
└── robot.cpp
```

Each can be compiled separately:

```text
rotation3.cpp  → rotation3.o
transform3.cpp → transform3.o
quaternion.cpp → quaternion.o
robot.cpp      → robot.o
```

Then the linker combines them:

```text
rotation3.o ───┐
transform3.o ──┤
quaternion.o ──┼──→ linker → final program/library
robot.o ───────┘
```

This is useful because if you modify only:

```text
rotation3.cpp
```

CMake usually doesn't need to recompile every `.cpp`.

It can just rebuild:

```text
rotation3.cpp → rotation3.o
```

and relink.

---

# 2. `.a` — static library

`.a` is a **static library archive**.

For example:

```text
librigid_body_kinematics.a
```

It is essentially a collection of object files:

```text
rotation3.o ───┐
transform3.o ──┼──→ librigid_body_kinematics.a
quaternion.o ──┘
```

The `a` stands for **archive**.

That's why your CMake installation has:

```cmake
ARCHIVE DESTINATION lib
```

which covers static libraries such as:

```text
librigid_body_kinematics.a
```

### What does "static" mean?

Suppose you build:

```text
my_robot_program
```

and it uses your static library:

```text
librigid_body_kinematics.a
```

At link time, the linker takes the required compiled code from the `.a` and incorporates it into the resulting executable.

Conceptually:

```text
my_program.o
     +
librigid_body_kinematics.a
     ↓
   linker
     ↓
my_robot_program
```

The resulting executable contains the needed library code.

Therefore, after the executable has been linked, it generally does **not** need that `.a` file at runtime.

---

# 3. `.so` — shared library

`.so` means **shared object**.

For example:

```text
librigid_body_kinematics.so
```

This is Linux's common format for a **shared/dynamic library**.

Windows has a roughly analogous concept:

```text
.dll
```

and macOS commonly uses:

```text
.dylib
```

The key difference from `.a` is that the library code is generally **not copied into every executable**.

Instead:

```text
program_A ──┐
            │
program_B ──┼──→ librigid_body_kinematics.so
            │
program_C ──┘
```

Multiple programs can use the same shared library.

---

## Static `.a` vs shared `.so`

Suppose two programs use your rotation library.

With a static library:

```text
librigid_body_kinematics.a
       │
       ├── required code copied/link-incorporated into → program_A
       │
       └── required code copied/link-incorporated into → program_B
```

The executables are more self-contained.

With a shared library:

```text
program_A ──┐
            ↓
       librigid_body_kinematics.so
            ↑
program_B ──┘
```

Both programs can use the external shared library.

This can reduce duplication, but now the `.so` must be available when the program runs.

---

## 4. Why does a library start with `lib`?

Linux convention commonly names libraries:

```text
lib<name>.so
lib<name>.a
```

So your CMake target:

```cmake
add_library(rigid_body_kinematics ...)
```

could correspond to:

```text
librigid_body_kinematics.so
```

or:

```text
librigid_body_kinematics.a
```

depending on how it's built.

The pieces are:

```text
lib rigid_body_kinematics .so
│   │                     │
│   │                     └─ shared library
│   └─────────────────────── library name
└─────────────────────────── conventional prefix
```

---

## 5. Compilation vs linking

This also introduces an important distinction.

### Compilation

The **compiler** converts each source file:

```text
.cpp → .o
```

For example:

```text
rotation3.cpp
      ↓ compiler
rotation3.o
```

### Linking

The **linker** connects compiled pieces together:

```text
.o + .o + libraries
          ↓
        linker
          ↓
 executable / shared library
```

For example:

```text
main.o ──────────────┐
                     │
controller.o ────────┼──→ linker → robot_program
                     │
rigid_body library ──┘
```

This explains a lot of C++ error messages.

A **compiler error** might be:

```text
'Rotation3' was not declared in this scope
```

The compiler couldn't understand/compile the source.

A **linker error** might be:

```text
undefined reference to
rigid_body_kinematics::Rotation3::identity()
```

That can mean the compiler knew from the header that:

```cpp
Rotation3::identity()
```

exists, but the linker couldn't find its compiled implementation.

---

## 6. How headers fit into this

`.hpp` is fundamentally different:

```text
.hpp   C++ declarations/source text
.cpp   C++ implementation/source text
.o     compiled object code
.a     static library
.so    shared library
```

For your project:

```text
rotation3.hpp
     │
     │ #include
     ↓
rotation3.cpp
     │
     │ compiler
     ↓
rotation3.o
     │
     │ linker/archiver
     ↓
librigid_body_kinematics.so
           or
librigid_body_kinematics.a
```

And then another program could look like:

```text
                    compile
controller.cpp ───────────────→ controller.o
      │
      │ #include
      ↓
rotation3.hpp


controller.o ─────────────┐
                          │
rigid_body_kinematics.so ─┼──→ link → controller executable
                          │
other libraries ──────────┘
```

So the key distinction is:

| File       | What it is                                           |
| ---------- | ---------------------------------------------------- |
| `.hpp`     | Header containing C++ declarations/definitions       |
| `.cpp`     | C++ source implementation                            |
| `.o`       | One source file compiled into object code            |
| `.a`       | Static library, typically an archive of object files |
| `.so`      | Shared/dynamic library                               |
| executable | Final runnable program                               |

For understanding CMake, the most important pipeline to remember is **`.cpp → .o → link → executable/library`**.


The distinction between **build**, **install**, and **export** is fundamental in CMake/ROS 2. A useful mental model is:

```text
Source code
    │
    │ BUILD
    ↓
Compiled library
    │
    │ INSTALL
    ↓
Organized installed package
    │
    │ EXPORT
    ↓
Usable by other CMake/ROS 2 packages
```

Let's use your `rigid_body_kinematics` package throughout.

## 1. Build — turn source code into a library

Your build starts here:

```cmake
add_library(${PROJECT_NAME}
  src/rotation3.cpp
)
```

`${PROJECT_NAME}` is:

```text
rigid_body_kinematics
```

so conceptually:

```cmake
add_library(rigid_body_kinematics
  src/rotation3.cpp
)
```

This creates a CMake **target** called `rigid_body_kinematics`.

CMake then needs information about how to compile it:

```cmake
target_compile_features(${PROJECT_NAME}
  PUBLIC cxx_std_17
)

target_compile_options(${PROJECT_NAME}
  PRIVATE
    "$<$<COMPILE_LANG_AND_ID:CXX,GNU,Clang>:-Wall;-Wextra;-Wpedantic>"
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC Eigen3::Eigen
)
```

These collectively say:

> Build `rigid_body_kinematics` using C++17, these warning flags, these header search paths, and Eigen.

### What actually happens?

You have:

```text
src/rotation3.cpp
```

which contains:

```cpp
#include "rigid_body_kinematics/rotation3.hpp"
```

The compiler first compiles it into an **object file**:

```text
rotation3.cpp
     │
     │ compiler
     ↓
rotation3.o
```

Then the object file is packaged/linked into your library:

```text
rotation3.o
     │
     │ linker / archiver
     ↓
librigid_body_kinematics.so
```

if you're building a shared library, or potentially:

```text
librigid_body_kinematics.a
```

for a static library.

So **build** means:

> Convert the source code into compiled artifacts.

---

## 2. Build directory

With ROS 2 you normally run:

```bash
colcon build
```

Suppose your workspace is:

```text
ros2_ws/
├── src/
│   └── rigid_body_kinematics/
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── include/
│       └── src/
│
├── build/
├── install/
└── log/
```

`colcon` uses the:

```text
build/
```

directory for intermediate build products.

For example:

```text
build/
└── rigid_body_kinematics/
    ├── CMakeFiles/
    ├── rotation3.o
    └── ...
```

You generally don't manually work with these files.

The build directory is essentially CMake's **workspace for compilation**.

---

# 3. Install — create the usable package layout

After building the library, you have compiled artifacts.

But they're still essentially build products.

You now want to organize the files into a standardized structure that other software can use.

That's what:

```cmake
install(...)
```

does.

You have two install commands.

### Install the library

```cmake
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
```

For example, the compiled Linux shared library gets placed under:

```text
install/
└── rigid_body_kinematics/
    └── lib/
        └── librigid_body_kinematics.so
```

### Install the headers

You also have:

```cmake
install(
  DIRECTORY include/
  DESTINATION include
)
```

This takes:

```text
source package:

include/
└── rigid_body_kinematics/
    └── rotation3.hpp
```

and installs it as something like:

```text
install/
└── rigid_body_kinematics/
    └── include/
        └── rigid_body_kinematics/
            └── rotation3.hpp
```

So now the installed package contains both major things a C++ library consumer needs:

```text
installed rigid_body_kinematics
│
├── include/
│   └── rigid_body_kinematics/
│       └── rotation3.hpp
│
└── lib/
    └── librigid_body_kinematics.so
```

This distinction is important:

```text
.hpp
 ↓
needed when COMPILING code that uses your library

.so / .a
 ↓
contains the compiled IMPLEMENTATION
```

---

# 4. Why install if we already built it?

Because **build files are not necessarily organized for consumption**.

The build tree might contain:

```text
build/
├── CMakeFiles/
├── Makefile
├── cmake_install.cmake
├── rotation3.cpp.o
├── generated files
├── temporary files
└── ...
```

That's CMake's internal workspace.

You don't want another package to depend on random paths inside that structure.

Instead, installation creates a clean interface:

```text
install/
├── include/
├── lib/
└── share/
```

Think of the distinction as:

```text
BUILD
"Produce the thing"

INSTALL
"Put the finished thing where users are supposed to find it"
```

---

# 5. Export — tell other packages how to use it

Now suppose you create another ROS 2 package:

```text
robot_controller
```

and want to use:

```cpp
#include "rigid_body_kinematics/rotation3.hpp"
```

It's not enough for the header and `.so` to physically exist.

CMake also needs information such as:

* What CMake target represents the library?
* Where are its headers?
* Where is its compiled library?
* What dependencies does it require?
* Does it require Eigen?

This is where **exporting** enters the picture.

Your code has:

```cmake
ament_export_targets(
  export_${PROJECT_NAME}
  HAS_LIBRARY_TARGET
)
```

and:

```cmake
ament_export_dependencies(
  eigen3_cmake_module
  Eigen3
)
```

This essentially tells downstream packages:

> `rigid_body_kinematics` provides a library target, and that library publicly depends on Eigen.

---

# 6. `install(... EXPORT ...)` and `ament_export_targets(...)`

These two pieces work together.

First:

```cmake
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ...
)
```

The important part is:

```cmake
EXPORT export_${PROJECT_NAME}
```

You're putting the installed target into an **export set** named:

```text
export_rigid_body_kinematics
```

Then:

```cmake
ament_export_targets(
  export_${PROJECT_NAME}
  HAS_LIBRARY_TARGET
)
```

tells ament to expose that export set to downstream packages.

Conceptually:

```text
CMake target:
rigid_body_kinematics
        │
        │ install(... EXPORT ...)
        ↓
export_rigid_body_kinematics
        │
        │ ament_export_targets(...)
        ↓
Available to downstream packages
```

---

# 7. Export dependencies

Your library publicly depends on Eigen:

```cmake
target_link_libraries(${PROJECT_NAME}
  PUBLIC Eigen3::Eigen
)
```

Remember why it's `PUBLIC`: your public header itself uses Eigen:

```cpp
#include <Eigen/Core>

class Rotation3
{
public:
    Eigen::Vector3d rotate_vector(...);

private:
    Eigen::Matrix3d matrix_;
};
```

So anyone using `Rotation3` also needs Eigen.

Therefore you export the dependency:

```cmake
ament_export_dependencies(
  eigen3_cmake_module
  Eigen3
)
```

You're effectively saying:

> When another ROS package uses `rigid_body_kinematics`, make sure its Eigen dependency is known too.

This is a **transitive dependency**.

```text
robot_controller
       │
       │ depends on
       ↓
rigid_body_kinematics
       │
       │ depends on
       ↓
     Eigen3
```

---

# 8. How another package eventually uses your library

Imagine:

```text
ros2_ws/src/
│
├── rigid_body_kinematics/
│   ├── include/
│   └── src/
│
└── robot_controller/
    └── src/
        └── controller.cpp
```

Your second package has:

```cpp
#include "rigid_body_kinematics/rotation3.hpp"
```

Its CMake configuration could locate your package:

```cmake
find_package(rigid_body_kinematics REQUIRED)
```

and then link the appropriate exported target.

CMake can obtain the required usage information from the installed/exported package rather than you manually writing hard-coded paths such as:

```cmake
include_directories(
  /home/chien/ros2_ws/src/rigid_body_kinematics/include
)
```

or:

```cmake
link_directories(
  /some/random/path/to/library
)
```

That's one of the major benefits of proper CMake targets and exports.

---

# 9. Build interface vs install interface now makes more sense

This part:

```cmake
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
```

exists precisely because your library exists in two different environments.

### During build

Your header actually lives here:

```text
source/
└── rigid_body_kinematics/
    └── include/
        └── rigid_body_kinematics/
            └── rotation3.hpp
```

so use:

```cmake
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
```

### After installation

The consumer shouldn't care where your original source repository was.

It uses:

```cmake
$<INSTALL_INTERFACE:include>
```

meaning the header location relative to the installed package.

So:

```text
              rigid_body_kinematics target
                         │
              ┌──────────┴──────────┐
              │                     │
          BUILDING              INSTALLED
              │                     │
              ↓                     ↓
source/include/               install/include/
```

---

# 10. The three stages in one example

Start with your source:

```text
rigid_body_kinematics/
├── CMakeLists.txt
├── include/
│   └── rigid_body_kinematics/
│       └── rotation3.hpp
└── src/
    └── rotation3.cpp
```

### Step 1 — Build

```text
rotation3.cpp
      ↓ compile
rotation3.o
      ↓ link/archive
librigid_body_kinematics.so
```

**Question answered:**

> How do I turn my C++ source into a library?

### Step 2 — Install

Organize the finished files:

```text
install/
└── rigid_body_kinematics/
    ├── include/
    │   └── rigid_body_kinematics/
    │       └── rotation3.hpp
    │
    ├── lib/
    │   └── librigid_body_kinematics.so
    │
    └── share/
        └── rigid_body_kinematics/
            └── CMake/package metadata...
```

**Question answered:**

> Where should the finished library, headers, and metadata live?

### Step 3 — Export

Generate/expose metadata describing:

```text
rigid_body_kinematics
    ├── here's my CMake target
    ├── here's how to find my headers
    ├── here's how to link my library
    └── I also depend on Eigen3
```

Then another package can say, conceptually:

```cmake
find_package(rigid_body_kinematics REQUIRED)
```

rather than knowing the physical details of your build tree.

**Question answered:**

> How can another CMake/ROS 2 package discover and correctly use my installed library?

So the shortest mental model is:

> **Build = create the binary. Install = put the binary + headers in their proper reusable locations. Export = publish the CMake information that tells other packages how to consume them.**
