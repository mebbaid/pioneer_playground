# pioneer playground

# Overview
 - [:orange_book: The general idea](#orange_book-some-theory-behind-the-code)
 - [:page_facing_up: Dependencies](#page_facing_up-dependencies)

# :orange_book: The general idea
This repo tests feedback linearization, transverse feedback linearization and dynamic transverse feedback linearization for the purpose of allowing the pioneer_p3dx robot to follow a given path (a circle).
It also tests those controllers when sampling is considered, and compare the results to emulation.


# :page_facing_up: Dependencies
1. **CMAKE:** Install the CMAKE build system 3.5 or higher [Cmake](https://cmake.org/install/)

2. **Eigne:** For linear algebra operations [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page#Download)

3. **inih:** To parse the ini configuration file(s) [inih](https://github.com/OSSystems/inih)

It is necessary to add the following corresponding `Env` variables in the `.bashrc` or `.bash_profile` the following lines:
```sh
export REMOTE_API=/path/where/CoppliaSim/programming
export Eigen=/path/where/you/unpacked
export inih=/path/where/you/installed/
```

See how to edit system `Env` variables in `Windows` [here](https://appuals.com/how-to-edit-environment-variables-in-windows-10) 

# :hammer: Build the suite
## Linux

```sh
git clone https://github.com/mebbaid/pioneer_playground.git
cd pioneer_playground
mkdir build && cd build
cmake ../
make
[sudo] make install
```
Notice: `sudo` is not necessary if you specify the `CMAKE_INSTALL_PREFIX`. 

## Windows
Follow the same instructions from the Powershell. One can also opt to use the ``CMake`` gui application.

# :computer: How to run the simulation
1. Open `myscene.ttt` with `vrep/CoppliaSim`

2. Run 
```sh
cd $/path/where/you/installed
pioneer_SDTFL config.ini
```
3. Follow instructions and select the controller you want to simulate.


