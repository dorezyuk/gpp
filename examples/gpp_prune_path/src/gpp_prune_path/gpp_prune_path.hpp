/*
 * MIT License
 *
 * Copyright (c) 2021 Dima Dorezyuk
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once

#include <gpp_interface/post_planning_interface.hpp>

#include <iterator>

namespace gpp_prune_path {

namespace internal {

/**
 * @brief inplace pruning for a random-access iterator-sequence
 *
 * The generated sequence will always contain the first and last pose.
 * Use it in combination with erase, to obtain a pruned sequence.
 *
 * Below some examples:
 *
 * @code{cpp}
 * // create some data
 * std::vector<int> d {1, 2, 3, 4, 5};
 *
 * // prune the data and drop redundant data
 * // d contains after this {1, 3, 5}
 * d.erase(prune(d.begin(), d.end(), 2), d.end());
 *
 * // create another array
 * std::vector<int> e{1, 2, 3, 4, 5, 6};
 *
 * // run again the same operation:
 * // e contains after this {1, 4, 6}
 * e.erase(prune(e.begin(), e.end(), 3), e.end());
 *
 * @endcode
 *
 * @tparam _Iter random access iterator
 * @param _begin begin of the sequence
 * @param _end end of the sequence
 * @param _step positive integer for skipping
 * @returns the end iterator of the pruned sequence
 */
template <typename _Iter>
_Iter
prune(_Iter _begin, _Iter _end, size_t _step) {
  if (_step < 2)
    return _end;
  // find the distance between _begin and end
  const auto distance = std::distance(_begin, _end);

  // we prune such that we always include the first and last element of the
  // sequence. if the distance is less than three, we cannot prune.
  if (distance < 2)
    return _end;

  // find the end of our source.
  // note: we denote the source (s) as the iterator from where we are copying
  // data; the destination is denoted with (d).
  const _Iter d_end = _begin + (distance + _step - 2) / _step;

  // inplace pruning.
  // note: don't use std::move, as it doesn't work for ros-msgs...
  for (_Iter s_begin = _begin; _begin != d_end; ++_begin, s_begin += _step)
    *_begin = *s_begin;

  // last element must be updated separatelly:
  *_begin++ = *std::prev(_end);

  return _begin;
}
}  // namespace internal

// short-cut
using gpp_interface::PostPlanningInterface;

/**
 * @brief post-processing class which will prune the given path.
 *
 * You can configure the skip-step via the ros-parameter "~/step".
 * Its value defaults to 2. Negative values will deactivate this plugin -
 * calling GppPrunePath::postProcess will be a noop and directly return true.
 *
 * The plugin fails, if the given path has less than 2 elements.
 * It succeeds in all other cases.
 *
 * See the README.md for futher details.
 */
struct GppPrunePath : public PostPlanningInterface {
  bool
  postProcess(const Pose &_start, const Pose &_goal, Path &_path,
              double &_cost) override;

  void
  initialize(const std::string &_name, Map *_map) override;

private:
  int step_ = 0;
};

}  // namespace gpp_prune_path
