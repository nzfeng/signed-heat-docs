## Running tests

All tests are stored in the [`test/`](https://github.com/nzfeng/signed-heat-3d/tree/main/test) subdirectory, which also contains a few small input files. We use [GoogleTest](https://google.github.io/googletest/) as a testing framework.

Compile and execute the tests using
```
cd signed-heat-3d/test
mkdir build && cd build
cmake ..
make -j
bin/shm_test
```
