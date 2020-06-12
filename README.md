# pioneer playground

# Overview
 - [:orange_book: The general idea](#orange_book-some-theory-behind-the-code)
 - [:page_facing_up: Dependencies](#page_facing_up-dependencies)

# :orange_book: The general idea
This repo tests feedback linearization, transverse feedback linearization and dynamic transverse feedback linearization for the purpose of allowing the pioneer_p3dx robot to follow a given path.
It also tests those controllers when sampling is considered, and compare the results to emulation.


# :page_facing_up: Dependencies
1. **CMAKE:** Install the CMAKE build system 3.5 or higher [Cmake](https://cmake.org/install/)

2. **Eigne:** For linear algebra operations [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page#Download)

3. **inih:** To parse the ini configuration file(s) [inih](https://github.com/OSSystems/inih)

