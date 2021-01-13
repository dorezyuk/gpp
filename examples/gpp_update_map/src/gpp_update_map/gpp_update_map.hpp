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

#include <gpp_interface/pre_planning_interface.hpp>

namespace gpp_update_map {

// short-cut
using gpp_interface::PrePlanningInterface;

/**
 * @brief Class will call updateMap on the passed map.
 *
 * This allows the user to stop ticked-based re-rendering of the costmap.
 * The plugin only fails, if the internal map_ is a nullptr.
 * In all other cases, it succeeds.
 *
 * See README.md for the inteface documentation.
 */
struct GppUpdateMap : public PrePlanningInterface {
  bool
  preProcess(Pose& _start, Pose& _goal) override;

  void
  initialize(const std::string& _name, Map* _map) override;

private:
  Map* map_ = nullptr;
};

}  // namespace gpp_update_map
