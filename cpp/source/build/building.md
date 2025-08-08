# Building

`signed-heat-3d` uses CMake for to configure the build system. The basic workflow for downloading and compiling `signed-heat-3d` via a terminal is:

```
git clone --recurse-submodules https://github.com/nzfeng/signed-heat-3d.git
cd signed-heat-3d
mkdir build && cd build
cmake ..
make -j
```
If you do not clone recursively, some submodules or sub-submodules will not clone. (In particular, you may get errors about missing `CMakeLists.txt` files.) Initialize/update these submodules by running `git submodule update --init --recursive` or `git submodule update --recursive`.

The above commands compile `signed-heat-3d` as a library, and do not build any executables.

You can add `signed-heat-3d` to an existing project's CMakeLists.txt using

```
add_subdirectory("path/to/signed-heat-3d") # replace with your path
target_link_libraries(your-project-target signed-heat-3d)
```

#### Example

The [sample project](https://github.com/nzfeng/signed-heat-demo-3d) demonstrates how to use `signed-heat-3d` within another project, using a build system and a GUI.

## Compile options

Linear solves ca be optionally accelerated using the algebraic multigrid library [AMGCL](https://github.com/ddemidov/amgcl), which requires Boost. If you do not want to use Boost, use `cmake -DSHM_NO_AMGCL=On` to compile to a program without AMGCL but with solve times \~5x slower (more or less for larger/smaller problems). Force use of AMGCL via `cmake -DSHM_NO_AMGCL=Off`. Boost can be installed on macOS using `brew install boost`, and the necessary modules on Ubuntu using

```
sudo apt-get -y update
sudo apt-get -y install libboost-dev libboost-test-dev libboost-program-options-dev libboost-serialization-dev
```

Windows users should probably follow the instructions on the [Boost website](https://www.boost.org/releases/latest/).
