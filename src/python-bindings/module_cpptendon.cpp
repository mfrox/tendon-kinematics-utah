#include "submodule_controller.h"
#include "submodule_tendon.h"

#include <pybind11/pybind11.h>

#include <sstream>

namespace py = pybind11;

PYBIND11_MODULE(cpptendon, m) {
  def_submodule_tendon(m);
  def_submodule_controller(m);
}
