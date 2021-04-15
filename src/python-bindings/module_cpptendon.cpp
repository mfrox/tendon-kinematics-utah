#include <cpptoml/toml_conversions.h>
#include <tendon/BackboneSpecs.h>
#include <tendon/TendonResult.h>
#include <tendon/TendonRobot.h>
#include <tendon/TendonSpecs.h>

#include <pybind11/eigen.h>    // auto convert between python and eigen types
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>      // auto convert between python and STL types

#include <Eigen/Core>

#include <sstream>
#include <string>
#include <vector>

namespace E = Eigen;
namespace py = pybind11;

namespace {

void def_class_BackboneSpecs(py::module &m) {
  py::class_<tendon::BackboneSpecs>(m, "BackboneSpecs",
      "Specs for the backbone on a tendon-actuated robot")
    .def(py::init<double, double, double, double, double, double>(),
        py::arg("L") = 0.2,
        py::arg("dL") = 0.005,
        py::arg("ro") = 0.01,
        py::arg("ri") = 0.0,
        py::arg("E") = 2.1e6,
        py::arg("nu") = 0.3)

    // attributes
    .def_readwrite("L", &tendon::BackboneSpecs::L,
                   "robot backbone length (meters)")
    .def_readwrite("dL", &tendon::BackboneSpecs::dL,
                   "backbone discretization length (meters)")
    .def_readwrite("ro", &tendon::BackboneSpecs::ro,
                   "outer backbone radius (meters)")
    .def_readwrite("ri", &tendon::BackboneSpecs::ri,
                   "inner backbone radius (meters) - for hollow backbones")
    .def_readwrite("E", &tendon::BackboneSpecs::E,
                   "Young's modulus (pascals = N/m^2)")
    .def_readwrite("nu", &tendon::BackboneSpecs::nu,
        "Poisson's ratio: response in directions orthogonal to\n"
        "uniaxial stress.  Relates young's modulus to shear\n"
        "modulus with isotropic materials.")

    // methods
    .def("to_toml", [](const tendon::BackboneSpecs &obj, const std::string &fname) {
          cpptoml::to_file(obj, fname);
        }, py::arg("filepath"), "save this object to a toml file")

    // static methods
    .def_static("from_toml",
        [](const std::string &fname) {
          return cpptoml::from_file<tendon::BackboneSpecs>(fname);
        }, py::arg("filepath"),
        "load a BackboneSpecs object from the given toml file")

    // python-specific added methods
    .def("__eq__",
        [](const tendon::BackboneSpecs &a, const tendon::BackboneSpecs &b) {
          return a == b;
        })
    .def("__repr__", [](const tendon::BackboneSpecs &obj) {
          std::ostringstream builder;
          builder << "BackboneSpecs("
                     "L: " << obj.L << ", "
                     "dL: " << obj.dL << ", "
                     "ro: " << obj.ro << ", "
                     "ri: " << obj.ri << ", "
                     "E: " << obj.E << ", "
                     "nu: " << obj.nu << ")";
          return builder.str();
        })
    .def("__str__", [](const tendon::BackboneSpecs &obj) {
          std::ostringstream builder;
          cpptoml::to_stream(builder, obj.to_toml());
          return builder.str();
        })
    ;
}

void def_class_TendonResult(py::module &m) {
  py::class_<tendon::TendonResult>(m, "TendonResult",
      "Results from calculating tendon shape")
    .def(py::init<std::vector<double>,
                  std::vector<E::Vector3d>,
                  std::vector<E::Matrix3d>,
                  double,
                  std::vector<double>>(),
        py::arg("t") = std::vector<double>{},
        py::arg("p") = std::vector<E::Vector3d>{},
        py::arg("R") = std::vector<E::Matrix3d>{},
        py::arg("L") = 0.020,
        py::arg("L_i") = std::vector<double>{})

    // attributes
    .def_readwrite("t", &tendon::TendonResult::t,
                   "parameterization value")
    .def_readwrite("p", &tendon::TendonResult::p,
                   "p[j] = backbone position at t[j]")
    .def_readwrite("R", &tendon::TendonResult::R,
                   "R[j] = rotation matrix from base frame at t[j]")
    .def_readwrite("L", &tendon::TendonResult::L,
                   "Current length of the backbone (meters)")
    .def_readwrite("L_i", &tendon::TendonResult::L_i,
                   "L_i[i] = current length of tendon i from the robot base")

    // methods
    .def("to_toml",
         [](const tendon::TendonResult &result, const std::string &fname) {
           cpptoml::to_file(result, fname);
         }, py::arg("filepath"), "save this object to a toml file")
    .def("rotate_z", &tendon::TendonResult::rotate_z, py::arg("theta"),
         "rotate this object about the base-frame's z-axis.\n"
         "recalculates the positions and rotations only.")

    // python-specific added methods
    .def("__str__", [](const tendon::TendonResult &result) {
          std::ostringstream builder;
          cpptoml::to_stream(builder, result.to_toml());
          return builder.str();
        })
    ;
}

void def_class_TendonRobot(py::module &m) {
  py::class_<tendon::TendonRobot>(m, "TendonRobot",
      "Representation of a tendon-actuated robot")
    .def(py::init<double, tendon::BackboneSpecs,
                  std::vector<tendon::TendonSpecs>, bool, bool>(),
        py::arg("r") = 0.015,
        py::arg("specs") = tendon::BackboneSpecs{},
        py::arg("tendons") = std::vector<tendon::TendonSpecs>{},
        py::arg("enable_rotation") = false,
        py::arg("enable_retraction") = false)

    // public attributes
    .def_readwrite("r", &tendon::TendonRobot::r,
        "robot radius (meters)")
    .def_readwrite("specs", &tendon::TendonRobot::specs,
        "backbone specifications")
    .def_readwrite("tendons", &tendon::TendonRobot::tendons, "tendons")
    .def_readwrite("enable_rotation", &tendon::TendonRobot::enable_rotation,
        "enable rotation control")
    .def_readwrite("enable_retraction", &tendon::TendonRobot::enable_retraction,
        "enable retraction control")

    // public methods
    .def("state_size", &tendon::TendonRobot::state_size,
        "size of the state vector used.  Equal to number of tendons plus the\n"
        "rotation and retraction dimensions (if enabled).")
    .def("random_state", &tendon::TendonRobot::random_state,
        "generate and return a random valid state")
    .def("forward_kinematics", &tendon::TendonRobot::forward_kinematics,
        py::arg("state"),
        "calculate the backbone positions for a given state")
    .def("home_shape",
        py::overload_cast<double>(&tendon::TendonRobot::home_shape,
                                        py::const_),
        py::arg("s_start") = 0.0,
        "shape at the zero tension config (efficiently computed)\n"
        "\n"
        "Note, s_start will be used even if retraction is disabled.")
    .def("home_shape",
        py::overload_cast<const std::vector<double>&>(
          &tendon::TendonRobot::home_shape, py::const_),
        py::arg("state"),
        "shape at the zero tension config (efficiently computed)\n"
        "\n"
        "Will pull the s_start value out of the state only if retraction is\n"
        "enabled, otherwise a value of 0.0 will be used.")
    .def("shape",
        py::overload_cast<const std::vector<double> &>(
          &tendon::TendonRobot::shape, py::const_),
        py::arg("state"),
        "computes and returns the robot shape from the given state.")
    .def("shape",
        py::overload_cast<const std::vector<double>&, double, double>(
          &tendon::TendonRobot::shape, py::const_),
        py::arg("tau"), py::arg("rotation"),
        py::arg("retraction"),
        "computes and returns the robot shape from the given controls.")
    .def("is_valid",
        py::overload_cast<const std::vector<double>&,
                                const tendon::TendonResult&>(
          &tendon::TendonRobot::is_valid, py::const_),
        py::arg("state"), py::arg("home_shape"),
        "returns True if the state is within valid tension and length limits")
    .def("is_valid",
        py::overload_cast<const std::vector<double>&,
                                const tendon::TendonResult&,
                                const tendon::TendonResult&>(
          &tendon::TendonRobot::is_valid, py::const_),
        py::arg("state"), py::arg("home_shape"),
        py::arg("shape"),
        "returns True if the state is within valid tension and length limits")
    .def("calc_dl", &tendon::TendonRobot::calc_dl,
        py::arg("home_tendon_lengths"), py::arg("tendon_lengths"),
        "calculates the change in tendon lengths from the home shape\n"
        "\n"
        "expected to pass in TendonResult.L_i for both arguments, one\n"
        "from the home shape and the other from the current shape")
    .def("is_within_length_limits",
        py::overload_cast<const std::vector<double>&,
                                const std::vector<double>&>(
          &tendon::TendonRobot::is_within_length_limits, py::const_),
        py::arg("home_tendon_lengths"), py::arg("tendon_lengths"),
        "returns True if the difference between the two tendon lengths are\n"
        "within the length limits.")
    .def("is_within_length_limits",
        py::overload_cast<const std::vector<double>&>(
          &tendon::TendonRobot::is_within_length_limits, py::const_),
        py::arg("dl"),
        "returns True if the given difference vector is within the length\n"
        "limits")
    .def("to_toml", [](const tendon::TendonRobot &obj, const std::string &fname) {
          cpptoml::to_file(obj, fname);
        }, py::arg("filepath"), "save this object to a toml file")

    // static methods
    .def_static("from_toml", [](const std::string &fname) {
          return cpptoml::from_file<tendon::TendonRobot>(fname);
        }, py::arg("filepath"),
        "load a TendonRobot object from the given toml file")

    // python-specific added methods
    .def("__eq__",
        [](const tendon::TendonRobot &a, const tendon::TendonRobot &b) {
          return a == b;
        })
    .def("__repr__", [](const tendon::TendonRobot &robot) {
          using tendon::operator<<;
          std::ostringstream builder;
          builder << robot;
          return builder.str();
        })
    .def("__str__", [](const tendon::TendonRobot &robot) {
          std::ostringstream builder;
          cpptoml::to_stream(builder, robot.to_toml());
          return builder.str();
        })
    ;
}

void def_class_TendonSpecs(py::module &m) {
  py::class_<tendon::TendonSpecs>(m, "TendonSpecs",
      "Specs for a tendon on a tendon-actuated robot")
    .def(py::init<E::VectorXd, E::VectorXd, double, double, double>(),
        py::arg("C") = E::VectorXd{},
        py::arg("D") = E::VectorXd{},
        py::arg("max_tension") = 20.0,
        py::arg("min_length") = -0.015,
        py::arg("max_length") = 0.035)

    // public attributes
    .def_readwrite("C", &tendon::TendonSpecs::C,
        "theta polynomial coefficients of tendon")
    .def_readwrite("D", &tendon::TendonSpecs::D,
        "r polynomial coefficients of tendons")
    .def_readwrite("max_tension", &tendon::TendonSpecs::max_tension,
        "maximum allowed tension on the tendon (Newtons)")
    .def_readwrite("min_length", &tendon::TendonSpecs::min_length,
        "minimum string extension allowed (meters)")
    .def_readwrite("max_length", &tendon::TendonSpecs::max_length,
        "maximum string extension allowed (meters)")

    // methods
    .def("is_straight", &tendon::TendonSpecs::is_straight,
        "return True if the tendon is routed straight",
        py::arg("eps") = 0.0)
    .def("is_helix", &tendon::TendonSpecs::is_helix,
        "return True if the tendon is routed in a helical shape",
        py::arg("eps") = 0.0)
    .def("to_toml", [](const tendon::TendonSpecs &obj, const std::string &fname) {
          cpptoml::to_file(obj, fname);
        }, py::arg("filepath"), "save this object to a toml file")

    // static methods
    .def_static("from_toml",
        [](const std::string &fname) {
          return cpptoml::from_file<tendon::TendonSpecs>(fname);
        }, py::arg("filepath"),
        "load a TendonSpecs object from a given toml file")

    // python-specific added methods
    .def("__eq__",
        [](const tendon::TendonSpecs &a, const tendon::TendonSpecs &b) {
          return a == b;
        })
    .def("__repr__", [](const tendon::TendonSpecs &obj) {
          std::ostringstream builder;
          using tendon::operator<<;
          builder << obj;
          return builder.str();
        })
    .def("__str__", [](const tendon::TendonSpecs &obj) {
          std::ostringstream builder;
          cpptoml::to_stream(builder, obj.to_toml());
          return builder.str();
        })
    ;
}

} // end of unnamed namespace

PYBIND11_MODULE(cpptendon, m) {
  def_class_BackboneSpecs(m);
  def_class_TendonSpecs(m);
  def_class_TendonRobot(m);
  def_class_TendonResult(m);
}
