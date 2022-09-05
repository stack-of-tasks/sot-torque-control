# Installation

1. Clone the git repository:
```
git clone --recursive https://github.com/stack-of-tasks/sot-torque-control.git
cd sot-torque-control
```

2. If you need it, switch to the devel branch
```
git checkout devel
```

3. Create the build directory and move there
```
mkdir _build-RELEASE
cd _build-RELEASE
```

4. Run cmake
```
cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=$DEVEL_DIR/openrobots
```
In Ubuntu 14.04 you may need to add `-DCMAKE_CXX_FLAGS="-std=c++11"`.

5. Build the package
```
make -j4
```

6. Install the package
```
make install
```
