#include "TendonResult.h"

#include "cpptoml/cpptoml.h"
#include "cpptoml/toml_conversions.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace E = Eigen;

namespace tendon {

void TendonResult::rotate_z(double theta) {
  E::Matrix3d rot = E::AngleAxisd(theta, E::Vector3d::UnitZ())
                    .toRotationMatrix();
  for (auto &pval : this->p) { pval = rot * pval; }
  for (auto &Rval : this->R) { Rval = rot * Rval; }
}

std::shared_ptr<cpptoml::table> TendonResult::to_toml() const {
  auto container = cpptoml::make_table();
  auto tbl = cpptoml::make_table();
  container->insert("tendon_result", tbl);

  if (!t.empty()) {
    tbl->insert("t", cpptoml::to_toml(t));
  }
  if (!p.empty()) {
    auto tbl_arr = cpptoml::make_table_array();
    for (auto &point : p) {
      auto sub_tbl = cpptoml::make_table();
      sub_tbl->insert("point", cpptoml::to_toml(point));
      tbl_arr->push_back(sub_tbl);
    }
    tbl->insert("p", tbl_arr);
  }
  if (!R.empty()) {
    tbl->insert("R", cpptoml::to_toml(R));
  }
  tbl->insert("L", L);
  if (!L_i.empty()) {
    tbl->insert("L_i", cpptoml::to_toml(L_i));
  }

  return container;
}

TendonResult TendonResult::from_toml(std::shared_ptr<cpptoml::table> tbl) {
  if (!tbl) { throw std::invalid_argument("null table given"); }

  auto container = tbl->get("tendon_result")->as_table();
  if (!container) {
    throw cpptoml::parse_exception(
        "Wrong type detected for 'tendon_result': not a table");
  }

  TendonResult result;

  // t
  if (container->contains("t")) {
    auto t = container->get("t")->as_array();
    if (!t) {
      throw cpptoml::parse_exception("Wrong type detected for 't'");
    }
    result.t = cpptoml::to_stdvec<double>(t);
  }

  // p
  if (container->contains("p")) {
    auto p = container->get("p")->as_table_array();
    if (!p) {
      throw cpptoml::parse_exception("Wrong type detected for 'p'");
    }
    for (auto sub_tbl : p->get()) {
      auto point = sub_tbl->get("point")->as_array();
      if (!point) {
        throw cpptoml::parse_exception("Wrong point type detected");
      }
      result.p.emplace_back(cpptoml::to_point(point));
    }
  }

  // R
  if (container->contains("R")) {
    auto R = container->get("R")->as_table_array();
    if (!R) {
      throw cpptoml::parse_exception("Wrong type detected for 'R'");
    }
    for (auto sub_tbl : R->get()) {
      result.R.emplace_back(cpptoml::to_matrix(sub_tbl));
    }
  }

  // L
  auto L = container->get("L")->as<double>();
  if (!L) {
    throw cpptoml::parse_exception("Wrong type detected for 'L'");
  }
  result.L = L->get();

  // L_i
  if (container->contains("L_i")) {
    auto L_i = container->get("L_i")->as_array();
    if (!L_i) {
      throw cpptoml::parse_exception("Wrong type detected for 'L_i'");
    }
    result.L_i = cpptoml::to_stdvec<double>(L_i);
  }

  return result;
}

} // end of namespace tendon
