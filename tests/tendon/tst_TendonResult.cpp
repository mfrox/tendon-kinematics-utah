/**
 * author:        Michael Bentley
 * email:         mikebentley15@gmail.com
 * date-created:  21 March 2020
 */

#include "tendon/TendonResult.h"
#include <cpptoml/cpptoml.h>
#include <cpptoml/toml_conversions.h>

#include <gtest/gtest.h>

using tendon::TendonResult;

namespace {

TendonResult create_results() {
  TendonResult result;
  result.t = {1.1, 2.2, 3.3, 4.4, 5.5};
  result.p = {{1,2,3}, {3,4,5}, {6,7,8}};
  Eigen::Matrix3d mat;
  mat << 1, 2, 3,
         4, 5, 6,
         7, 8, 9;
  result.R.push_back(mat);
  mat << 2, 3, 4,
         5, 6, 7,
         8, 9, 0;
  result.R.push_back(mat);
  mat << 3, 4, 5,
         6, 7, 8,
         9, 0, 1;
  result.R.push_back(mat);
  result.L = 3.145;
  result.L_i = {2.1, 3.1, 4.1, 5.1};
  return result;
}

} // end of unnamed namespace

TEST(TendonResultTests, rotate_z_zero) {
  auto result = create_results();
  auto actual = create_results();
  result.rotate_z(0);
  ASSERT_EQ(result.t  , actual.t  );
  ASSERT_EQ(result.p  , actual.p  );
  ASSERT_EQ(result.R  , actual.R  );
  ASSERT_EQ(result.L  , actual.L  );
  ASSERT_EQ(result.L_i, actual.L_i);
}

TEST(TendonResultTests, rotate_z_90_degrees) {
  auto result = create_results();
  Eigen::Matrix3d rot;
  rot << 0, -1, 0,
         1,  0, 0,
         0,  0, 1;
  for (auto &p : result.p) { p = rot * p; }
  for (auto &R : result.R) { R = rot * R; }

  auto actual = create_results();
  actual.rotate_z(M_PI / 2);

  ASSERT_EQ(result.t  , actual.t  );
  ASSERT_EQ(result.L  , actual.L  );
  ASSERT_EQ(result.L_i, actual.L_i);

  size_t N = result.p.size();
  ASSERT_EQ(N, result.p.size());
  ASSERT_EQ(N, result.R.size());
  ASSERT_EQ(N, actual.p.size());
  ASSERT_EQ(N, actual.R.size());
  for (size_t i = 0; i < N; ++i) {
    auto &rp = result.p[i];
    auto &ap = actual.p[i];
    auto &rR = result.R[i];
    auto &aR = actual.R[i];

    ASSERT_DOUBLE_EQ(rp[0], ap[0]);
    ASSERT_DOUBLE_EQ(rp[1], ap[1]);
    ASSERT_DOUBLE_EQ(rp[2], ap[2]);

    ASSERT_DOUBLE_EQ(rR(0, 0), aR(0, 0));
    ASSERT_DOUBLE_EQ(rR(0, 1), aR(0, 1));
    ASSERT_DOUBLE_EQ(rR(0, 2), aR(0, 2));
    ASSERT_DOUBLE_EQ(rR(1, 0), aR(1, 0));
    ASSERT_DOUBLE_EQ(rR(1, 1), aR(1, 1));
    ASSERT_DOUBLE_EQ(rR(1, 2), aR(1, 2));
    ASSERT_DOUBLE_EQ(rR(2, 0), aR(2, 0));
    ASSERT_DOUBLE_EQ(rR(2, 1), aR(2, 1));
    ASSERT_DOUBLE_EQ(rR(2, 2), aR(2, 2));
  }
}

TEST(TendonResultTests, to_toml_default) {
  TendonResult result;
  auto actual = TendonResult::from_toml(TendonResult().to_toml());
  ASSERT_EQ(result.t  , actual.t  );
  ASSERT_EQ(result.p  , actual.p  );
  ASSERT_EQ(result.R  , actual.R  );
  ASSERT_EQ(result.L  , actual.L  );
  ASSERT_EQ(result.L_i, actual.L_i);
}

TEST(TendonResultTests, to_toml) {
  auto result = create_results();
  auto actual = TendonResult::from_toml(result.to_toml());
  ASSERT_EQ(result.t  , actual.t  );
  ASSERT_EQ(result.p  , actual.p  );
  ASSERT_EQ(result.R  , actual.R  );
  ASSERT_EQ(result.L  , actual.L  );
  ASSERT_EQ(result.L_i, actual.L_i);
}

TEST(TendonResultTests, to_toml_default_through_string) {
  TendonResult result;
  auto str = cpptoml::to_string(result.to_toml());
  std::istringstream in(str);
  cpptoml::parser toml_parser(in);
  auto actual = TendonResult::from_toml(toml_parser.parse());
  ASSERT_EQ(result.t  , actual.t  );
  ASSERT_EQ(result.p  , actual.p  );
  ASSERT_EQ(result.R  , actual.R  );
  ASSERT_EQ(result.L  , actual.L  );
  ASSERT_EQ(result.L_i, actual.L_i);
}

TEST(TendonResultTests, to_toml_through_string) {
  auto result = create_results();
  auto str = cpptoml::to_string(result.to_toml());
  std::istringstream in(str);
  cpptoml::parser toml_parser(in);
  auto actual = TendonResult::from_toml(toml_parser.parse());
  ASSERT_EQ(result.t  , actual.t  );
  ASSERT_EQ(result.p  , actual.p  );
  ASSERT_EQ(result.R  , actual.R  );
  ASSERT_EQ(result.L  , actual.L  );
  ASSERT_EQ(result.L_i, actual.L_i);
}

TEST(TendonResultTests, to_toml_no_time_skips_table_array) {
  auto result = create_results();
  result.t.clear();
  auto tbl = result.to_toml();
  ASSERT_FALSE(tbl->contains("t"));
}

TEST(TendonResultTests, to_toml_no_points_skips_table_array) {
  auto result = create_results();
  result.p.clear();
  auto tbl = result.to_toml();
  ASSERT_FALSE(tbl->contains("p"));
}

TEST(TendonResultTests, to_toml_no_rotations_skips_table_array) {
  auto result = create_results();
  result.R.clear();
  auto tbl = result.to_toml();
  ASSERT_FALSE(tbl->contains("R"));
}

TEST(TendonResultTests, to_toml_no_lengths_skips_table_array) {
  auto result = create_results();
  result.L_i.clear();
  auto tbl = result.to_toml();
  ASSERT_FALSE(tbl->contains("L_i"));
}

TEST(TendonResultTests, from_toml_nullptr) {
  ASSERT_THROW(TendonResult::from_toml(nullptr), std::invalid_argument);
}

TEST(TendonResultTests, from_toml_empty) {
  auto tbl = cpptoml::make_table();
  ASSERT_THROW(TendonResult::from_toml(tbl), std::out_of_range);
}

TEST(TendonResultTests, from_toml_in_container) {
  auto result = create_results();
  auto tbl = result.to_toml();
  ASSERT_TRUE(tbl->contains("tendon_result"));
  auto actual = TendonResult::from_toml(tbl);
  ASSERT_EQ(result.t  , actual.t  );
  ASSERT_EQ(result.p  , actual.p  );
  ASSERT_EQ(result.R  , actual.R  );
  ASSERT_EQ(result.L  , actual.L  );
  ASSERT_EQ(result.L_i, actual.L_i);
}

TEST(TendonResultTests, from_toml_missing_t) {
  auto tbl = create_results().to_toml();
  tbl->get("tendon_result")->as_table()->erase("t");
  auto result = TendonResult::from_toml(tbl);
  ASSERT_TRUE(result.t.empty());
}

TEST(TendonResultTests, from_toml_missing_p) {
  auto tbl = create_results().to_toml();
  tbl->get("tendon_result")->as_table()->erase("p");
  auto result = TendonResult::from_toml(tbl);
  ASSERT_TRUE(result.p.empty());
}

TEST(TendonResultTests, from_toml_missing_R) {
  auto tbl = create_results().to_toml();
  tbl->get("tendon_result")->as_table()->erase("R");
  auto result = TendonResult::from_toml(tbl);
  ASSERT_TRUE(result.R.empty());
}

TEST(TendonResultTests, from_toml_missing_L) {
  auto tbl = create_results().to_toml();
  tbl->get("tendon_result")->as_table()->erase("L");
  ASSERT_THROW(TendonResult::from_toml(tbl), std::out_of_range);
}

TEST(TendonResultTests, from_toml_missing_L_i) {
  auto tbl = create_results().to_toml();
  tbl->get("tendon_result")->as_table()->erase("L_i");
  auto result = TendonResult::from_toml(tbl);
  ASSERT_TRUE(result.L_i.empty());
}

TEST(TendonResultTests, from_toml_wrong_type_t) {
  auto tbl = create_results().to_toml();
  tbl->get("tendon_result")->as_table()->insert("t", 5);
  ASSERT_THROW(TendonResult::from_toml(tbl), cpptoml::parse_exception);
}

TEST(TendonResultTests, from_toml_wrong_type_p) {
  auto tbl = create_results().to_toml();
  tbl->get("tendon_result")->as_table()->insert("p", "name");
  ASSERT_THROW(TendonResult::from_toml(tbl), cpptoml::parse_exception);
}

TEST(TendonResultTests, from_toml_wrong_type_R) {
  auto tbl = create_results().to_toml();
  tbl->get("tendon_result")->as_table()->insert("R", 25);
  ASSERT_THROW(TendonResult::from_toml(tbl), cpptoml::parse_exception);
}

TEST(TendonResultTests, from_toml_wrong_type_L) {
  auto tbl = create_results().to_toml();
  tbl->get("tendon_result")->as_table()->insert("L", "name");
  ASSERT_THROW(TendonResult::from_toml(tbl), cpptoml::parse_exception);
}

TEST(TendonResultTests, from_toml_wrong_type_L_i) {
  auto tbl = create_results().to_toml();
  tbl->get("tendon_result")->as_table()->insert("L_i", 5);
  ASSERT_THROW(TendonResult::from_toml(tbl), cpptoml::parse_exception);
}
