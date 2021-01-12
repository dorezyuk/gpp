#include <gpp_prune_path/gpp_prune_path.hpp>
#include <gtest/gtest.h>

#include <numeric>

using namespace gpp_prune_path::internal;
// file contains unit tests for the prune function

TEST(prune, empty) {
  std::vector<int> d;
  EXPECT_EQ(d.end(), prune(d.begin(), d.end(), 3));
}

TEST(prune, less_then_two) {
  std::vector<int> d(1);
  EXPECT_EQ(d.end(), prune(d.begin(), d.end(), 2));
}

TEST(prune, exactly_two) {
  std::vector<int> d(2);

  for (size_t ii = 0; ii != 10; ++ii)
    EXPECT_EQ(d.end(), prune(d.begin(), d.end(), ii));
}

namespace {
// dont polute the namespace

/// @brief parameter for the test
struct param {
  param(size_t _size, const std::vector<int>& _d) : size(_size), expected(_d) {}
  size_t size;                ///< size of the iota-filled vector
  std::vector<int> expected;  ///< expected outcome
};

/// @brief fixture for the test
struct fixture : public testing::TestWithParam<param> {};

INSTANTIATE_TEST_SUITE_P(prune, fixture,
                         testing::Values(param(4, {0, 3}), param(5, {0, 3, 4}),
                                         param(10, {0, 3, 6, 9}),
                                         param(11, {0, 3, 6, 9, 10}),
                                         param(12, {0, 3, 6, 9, 11})));

TEST_P(fixture, regression) {
  // get the parameter
  const auto p = GetParam();

  // setup the input vector (d as data)
  std::vector<int> d(p.size);
  std::iota(d.begin(), d.end(), 0);

  // run our test with the skip size 3
  const int step = 3;
  const auto end = prune(d.begin(), d.end(), step);

  // check first the size
  ASSERT_EQ(p.expected.size(), std::distance(d.begin(), end));

  // check the members
  auto kk = p.expected.begin();
  for (auto ii = d.begin(); ii != end; ++ii, ++kk)
    EXPECT_EQ(*ii, *kk);
}
}  // namespace

int
main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
