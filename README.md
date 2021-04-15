# Tendon Kinematics Export

This code is an export of our `tendon_experiments` code repository,
specifically stripped to only contain the pieces that can be useful to one of
our callaborators, Margaret Rox.


## Matlab Code

The original Matlab code given to us by Caleb Rucker is found in the `matlab`
folder, along with a refactored one for more readability (and less
performance).


## Robot Configurations

In the `config` directory, an example configuration file is given to specify a
tendon-driven robot.  This type of file can be loaded in the C++ code with
`cpptoml::from_file()` (see the C++ Code section below).  The main sections are

- `backbone_specs`: values to be loaded into a `tendon::BackboneSpecs` object
- `tendons`: a list (hence more then one specified) of tendons, each will be
  loaded into a `tendon::TendonSpecs` object
- `tendon_robot`: the remaining fields for a `tendon::TendonRobot` object.  To
  load a `tendon::TendonRobot` object, you also need a list of `tendons` groups
  and a `backbone_specs` group.

Look into the header files for each of these types to see what each field
represents.


## C++ Code

The C++ code is found in the `src` directory.  It is broken down below and
explained.

`tendon/`: the robot specification and tendon forward kinematic modeling code.  The notable functionality is:

- `tendon::TendonRobot` class: this is the main class describing a
  tendon-driven robot.  Each tendon is a `tendon::TendonSpecs` object, and the backbone
  is described as a `tendon::BackboneSpecs` object.
  - field `r`: Radius of the robot for collision-checking (e.g., the radius of
    the disks around the backbone)
  - field `specs`: the `tendon::BackboneSpecs` object describing the robot
    backbone
  - field `tendons`: a `std::vector` of `tendon::TendonSpecs` objects,
    describing the tendons
  - field `enable_rotation`: true means that the robot is capable of rotating
    about its base in addition to being actuated by tendons.  This simply cause
    the results of the shape computation to be rotated about the z-axis
    afterwards.
  - field `enable_retraction`: true means that the robot is capable of being
    inserted and retracted into the environment.  This assumes that the point
    of insertion is fixed (i.e., that the position and orientation are constant
    at the point of insertion), and therefore simply computes the shape from
    that insertion point, ignoring the portion of the robot not inserted into
    the environment.  For a physical robot, this may be achieved with a rigid
    sheath until the insertion point or a robotic arm holding the tendon-driven
    robot that maintains this invariant.  The retraction range is assumed to be
    from zero (fully inserted in the environment) to the full length of the
    robot (the tip is at the insertion point)
  - method `state_size()`: returns the size of the control vector.  The control
    vector is first the tensions, then rotation (if enabled), then retraction
    (if enabled)
  - method `random_state()`: randomly samples a control vector within the
    bounds on the controls.
  - method `shape()`: given a control vector, return the shape computation as a
    `tendon::TendonResult` object.
  - method `home_shape()`: more efficiently computes the `tendon::TendonResult`
    object for the shape computation at the home position, which is the zero
    vector.  You can optionally give a retraction value or a state vector for
    it to calculate the home shape but with a specified retraction value other
    than zero.
  - method `is_valid()`: returns true if the given configuration and subsequent
    computed shape is within both tension and length limits.
  - method `calc_dl()`: given two tendon length vectors (like the `L_i` vector
    from a `tendon::TendonResult` object), calculate the change in length from
    home position of each tendon.  A positive value indicates pulling, whereas
    a negative value indicates giving more slack
  - method `to_toml()` and `from_toml()`: ways to save and load configuration
    files with all of the details of the `tendon::TendonRobot`.  You usually
    don't use these directly.  Instead, it is easier to use convenience
    functions from `cpptoml/toml_conversions.h` like `cpptoml::to_file()` and
    `cpptoml::from_file()`.  For example, to load a `tendon::TendonRobot` to an
    object named `robot`, you would call `auto robot =
    cpptoml::from_file<tendon::TendonRobot>(filename)`, and to save the `robot`
    object to a file, you would call `cpptoml::to_file(robot, filename)`.  This
    functionality is available for all classes that implement `to_toml()` and
    `from_toml()`.
  - method `forward_kinematics()`: a convenience for `shape()` that throws away
    all other shape information and just returns the calculated points along
    the backbone.
- `tendon::BackboneSpecs`: specifications of a tendon-robot backbone.  The
  fields are described in the header file `tendon/BackboneSpecs.h`.  Just look
  there.  Used by `tendon::TendonRobot`.  A notable field is
  - field `dL`: the discretization of the backbone used in the integration in
    the shape computation.  Smaller values make the model slower but more
    accurate.
- `tendon::TendonSpecs`: specifications of a single tendon for a tendon-driven
  robot.  The fields are described in the header file `tendon/TendonSpecs.h`.
  Used by `tendon::TendonRobot`.
- `tendon::TendonResult`: the return type of a tendon-robot shape computation.
  The five fields are described in detail in the header file
  `tendon/TendonResult.h`.

`apps`: Some example command-line applications to be useful and to show how to
use this library.

- `tendon_shape_example`: Example of how to hard-code a robot specification and
  perform a shape computation.  Provides timing of running a 3-tendon robot
  under a single set of tensions using OpenMP.
- `estimate_length_limits`: Loading from a robot toml specification file,
  randomly sample configurations and estimate tendon length limits associated
  with the tension limits found in that file.  Print the estimates to the
  console.
- `estimate_tension_limits`: Loading from a robot toml specification file,
  sample configurations and estimate tension limits associated with the length
  limits found in that file.  Print the estimates to the console.

`cliparser`: Command-line parser.  It only has one class called `CliParser`.
Look at the example apps to see examples of how it is used.  You are welcome to
use it if you'd like.  It is bundled here simply to allow you to compile the
provided command-line apps.

`cpptoml`: Functionality for saving to and loading from toml configuration
files (see https://github.com/toml-lang/toml).  The `cpptoml/cpptoml.h` file is
a 3rd-party library imported into this code base (and minimally modified).  The
`cpptoml/toml_conversions.h` is a file with some convenience functions that we
wrote.  Of note are the following functions

  - method `to_toml()` and `from_toml()`: ways to save and load configuration
    files with all of the details of the `tendon::TendonRobot`.  You usually
    don't use these directly.  Instead, it is easier to use convenience
    functions from `cpptoml/toml_conversions.h` like `cpptoml::to_file()` and
    `cpptoml::from_file()`.  For example, to load a `tendon::TendonRobot` to an
    object named `robot`, you would call `auto robot =
    cpptoml::from_file<tendon::TendonRobot>(filename)`, and to save the `robot`
    object to a file, you would call `cpptoml::to_file(robot, filename)`.  This
    functionality is available for all classes that implement `to_toml()` and
    `from_toml()`.
- `cpptoml::to_file()`: given an object (that has an implemented `to_toml()`
  method) and a filename, write the object to that file, overwriting existing
  content if the file already exists.
- `cpptoml::from_file<Type>()`: load a toml file into a specific object type
  (but only if that type implements a `from_toml()` method).  For example,
  `auto robot = cpptoml::from_file<tendon::TendonRobot>(filename);` will load a
  `tendon::TendonRobot` object from the file.
- `cpptoml::to_stream()`: take a toml object and output it to a specific stream
  (like the console or an open file).
- `cpptoml::to_string()`: convert a toml object to a `std::string`.

`csv`: classes for reading from and writing to CSV files (comma-separated
format).  All classes are declared inside of the `csv/Csv.h` header file.

- `csv::CsvReader`: Takes in a C++ input stream `std::istream`, assumes there
  is a header row, and provides the ability to parse into `csv::CsvRow`
  objects.
  - method `bool()`: returns true when you've finished reading the file
  - method `header()`: returns a pointer to a `csv::CsvRow` object representing
    the header row.
  - method `stream()`: returns the stream that was passed to the constructor
  - method `operator>>()`: the way to get the next `csv::CsvRow` object
- `csv::CsvWriter`: Takes in a C++ output stream `std::ostream`, and gives
  functionality to write to a CSV file.
  - method `write_row()`: given a `std::vector` of a single type, write it as a
    new row to the file (calling `new_row()` after it's done writing each
    element).
  - method `operator<<()`: append this object as a new value on the current
    row.  Supported types are `int`, `long`, `long long`, `unsigned int`,
    `unsigned long`, `unsigned long long`, `float`, `double`, `long double`,
    and `std::string`.
  - method `new_row()`: mark the end of the current row.  Writes a newline to
    the file and resets the internal state to account for the first element in
    the row.
- `csv::CsvRow`: a single row returned by `csv::CsvReader`.  It derives from
  `std::vector<std::string>`, so you can treat it like a normal vector.
  - method `header()`: return a pointer to the `csv::CsvRow` object
    representing the header row.
  - method `setHeader()`: set the header row object
  - method `operator[]`: aside from the normal indexing you can do with
    integers (inherited from `std::vector`), you can also pass in a column name
    `std::string` value into the square brackets to return the value under that
    header's column.

Here is an example of using these CSV classes, which simply reads in a csv file
and writes the columns `A` and `B` out, and also adds an additional column at the
beginning with the row index.  This assumes the input CSV file has columns `A`
and `B`, and will ignore all other columns.

```{c++}
#include <csv/Csv.h>
#include <util/openfile_check.h>

#include <fstream>

int main() {
  // open "in.csv" and "out.csv" files
  std::ifstream in;
  std::ofstream out;
  util::openfile_check(in, "in.csv");
  util::openfile_check(out, "out.csv");

  csv::CsvReader reader(in);
  csv::CsvWriter writer(out);

  // write header
  writer << "idx" << "A" << "B";
  writer.new_row();

  csv::CsvRow row;
  int idx = 1;
  while (reader >> row) {
    writer << idx << row["A"] << row["B"];
    writer.new_row();
    idx++;
  }

  return 0;
}
```

`util`: a directory of miscelaneous utilities.

- `openfile_check.h`: only provides the `util::openfile_check()` convenience
  function.
  - `util::openfile_check()`: a convenience function for opening files into
    standard input and output streams with checks to make sure you have
    permissions (e.g., if you try to open a file that doesn't exist, throw an
    exception).

Example of `util::openfile_check()` which opens an input file, and will throw a
`std::ios_base::failure` exception if it cannot be opened.

```{c++}
std::ifstream in;
util::openfile_check(in, filename);
```

- `macros.h`: just provides the `UNUSED_VAR()` macro function to get rid of
  unused variable compiler warnings.  It makes clear in the code that you meant
  to not use that value.
- `vector_ops.h`: convenience functions for `std::vector<double>` and Eigen
  vectors.  Notable functions are below.
  - `util::range()`: an interleaving is generated between two points by a
    discretization.
  - `util::linspace()`: similar to Matlab's `linspace()` that works with
    `std::vector<double>`.
- `poly.h`: functions for working with polynomials.  Used by the shape
  computation to calculate the tendon polynomials.
  - `util::poly_at()`: similar to Matlab's `poly_at()` which calculates the
    value of a polynomial with given coefficients at a specified parameter
    value.
  - `util::poly_der()`: similar to Matlab's `poly_der()` which calculates the
    polynomial coefficients of the derivative of the given polynomial
    coefficients.


## Python Bindings

Within the C++ source code, there is a `python-bindings` directory.  This
contains python bindings for part of the code.

The python module that is generated is called `cpptendon`.

You can either use this module from the build directory, or you can add your
build directory to your `PYTHONPATH`

```
cd build
export PYTHONPATH=$PYTHONPATH:$PWD
```

Then you can import and use the python module.  For example:

```
import cpptendon
robot = cpptendon.TendonRobot.from_toml('my-robot.toml')
controls = [1.0] * robot.state_size()
home_shape = robot.home_shape()
shape = robot.shape(controls)
dl = robot.calc_dl(home_shape.L_i, shape.L_i)
robot.to_toml('my-robot-copy.toml')
```

Note that to load from file, you use the static method `from_toml()` and to
save to file you would use the method `to_toml()`.

Right now, this module only has the tendon shape computations implemented.
