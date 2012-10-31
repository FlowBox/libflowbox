libflowbox
==========
FlowBox C++ Library for processing flow data

## Build Status
* master: 
   [![Build Status master](https://secure.travis-ci.org/FlowBox/libflowbox.png?branch=master)](http://travis-ci.org/FlowBox/libflowbox)
* develop: 
   [![Build Status develop](https://secure.travis-ci.org/FlowBox/libflowbox.png?branch=develop)](http://travis-ci.org/FlowBox/libflowbox)

Prerequisites (3th party dependencies)
--------------------------------------------------------------------------------
* compiler
* cmake
* BZip2 library installed  	(binary version and headers)
* ZLIB library installed 	(binary version and headers)
* pthread library installed	(binary version and headers)
* math library installed (binary version and headers)

Note: For the headers, you might need the -dev versions of the
packages for your OS.

For Ubuntu, you need to install:

```
sudo apt-get install build-essential cmake zlib1g-dev libbz2-dev
```

Documentation
--------------------------------------------------------------------------------
You can create the documentation for this library using
doxygen as follows:

```doxygen FlowBox.doxyfile```

This generates the documentation in the doc/ directory.
The HTML documentation can be opened by accessing the
index.html file in doc/html/index.html.


Building the library in the lib/ subdirectory
--------------------------------------------------------------------------------
If you want to modify the library, you probably want to
also create a project file for an IDE. How to do this for
eclipse, check the corresponding section in this README file.

```
cd lib
cmake -G"Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug ../
make
```
### NOTE:
If you want to see a detailed build output, use
make `VERBOSE=X` with X as a number indicating the level
of detail. 1 is enough to see the compiler command line output.

If you want to build the deb, rpm and archive packages:
`make packages`

If you want to install the library into the default library
directory of your system (e.g., /usr/local/lib), do:
`make install`


Building in an arbitrary directory (generic out-of-source tree builds)
--------------------------------------------------------------------------------
### Assumptions:
* Source tree is in `<SRC_TREE>`
* Build directory is `<BUILD_DIR>`

1. Create a directory for building the library or application
   You can do an in-path build, but it is recommanded to keep
   the source tree separate from the build tree.

   ```
   mkdir <BUILD_DIR>
   cd <BUILD_DIR>
   ```

2. Rename override.cmake.example to override.cmake
   and edit it, if you want to change/adapt the compiler flags
   for DEBUG,RELEASE,... etc. builds.

3. Prepare for building the FlowBox library.
   Note: DCMAKE_BUILD_TYPE=<Debug,Release,...>.
   WARNING: <absolute path ...> might not have a "/" at the end!

   ```
   cmake -G"Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug <SRC_TREE>
   ````

4. Build with: `make all`

Creating a Eclipse project from source
--------------------------------------------------------------------------------

Check: http://www.batchmake.org/Wiki/Eclipse_CDT4_Generator
for a simple HOWTO.

### Note: The cmake command to use for this purpose is

   `cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug <SRC_TREE>`

### Hints for working with Eclipse:
1. For the indexer to work with STL types too, please add the STL
   include directory (compiler specific):

   Project -> Properties -> C/C++ Include Paths -> Add External Include Path

   Under Linux, this is usually something like:
   /usr/include/c++/<Compiler Version>

