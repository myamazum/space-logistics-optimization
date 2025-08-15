This instruction is only for system-wide installation on Linux.
For more detailed and complete instruction, see [the official installation guide](https://www.scipopt.org/doc/html/md_INSTALL.php)

## Installation prerequisites

```sh
sudo apt install wget cmake g++ m4 xz-utils libgmp-dev unzip zlib1g-dev libboost-program-options-dev libboost-serialization-dev libboost-regex-dev libboost-iostreams-dev libtbb-dev libreadline-dev pkg-config git liblapack-dev libgsl-dev flex bison libcliquer-dev gfortran file dpkg-dev libopenblas-dev rpm
```

If not on ubuntu/debian distro, swap `apt` with `yum`, `pacman`, etc.

## Download and unzip

Go to the [download page](https://scipopt.org/index.php#download) download `scipoptsuite-X.X.X.tgz`.
Make sure to select the correct OS and SCIP version.
`cd` to where the raball file is, and unzip it (here we assume version `9.1.0`)

```sh
tar xvzf scipoptsuite-9.1.0.tgz
```

## Compile with Makefile

Another way to compile SCIP is by `CMake` (see the official doc), but we're sticking to makefile here

```sh
cd scipoptsuite*/
make # compile SCIP
make test # test complied SCIP
sudo make install # if you want to install it in a specific dir, add INSTALLDIR=/desired/installation/path
```

Check your installation with

```sh
which scip
```

Then check it is usable in `pyomo` by running

```sh
pyomo help --solvers
```
