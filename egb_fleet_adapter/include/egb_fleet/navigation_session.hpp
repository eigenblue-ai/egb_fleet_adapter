// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 The EgbFleetAdapter Authors
// Licensed under the Apache License, Version 2.0

#ifndef EGB_FLEET__NAVIGATION_SESSION_HPP_
#define EGB_FLEET__NAVIGATION_SESSION_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <optional>
#include <vector>

namespace egb_fleet {

/**
 * Represents a single plan execution lifecycle.
 *
 * Threading model:
 *   - RMF thread: creates sessions, calls follow_new_path/stop, reads pending
 * feedback
 *   - Zenoh feedback thread: writes pending feedback (waypoint index + ETA)
 *   - Zenoh status thread: reads goal_id, invokes completion/failure callbacks
 *   - Timer thread (update loop): drains pending feedback → calls
 * rmf_arrival_estimator
 *
 * CRITICAL: rmf_arrival_estimator must NEVER be called from the Zenoh thread.
 * RMF may free the plan data that the estimator references at any time on the
 * RMF thread. The only safe place to call it is from the timer/update thread
 * which runs on the same executor as RMF, ensuring sequential access.
 */
struct NavigationSession {

  mutable std::mutex mutex;

  // --- Goal tracking ---
  std::optional<std::array<uint8_t, 16>> goal_id;
  std::atomic<bool> done{false};

  // --- Callbacks ---
  std::function<void()> completion_callback;
  std::function<void()> failure_callback;
  std::function<void(size_t, double)>
      arrival_estimator; // NavigationController's version

  // RMF's arrival estimator — called from timer thread only, never from Zenoh.
  using RmfArrivalEstimator =
      std::function<void(std::size_t, std::chrono::nanoseconds)>;
  RmfArrivalEstimator rmf_arrival_estimator;

  // --- Buffered feedback from Zenoh thread ---
  // Zenoh writes here, timer thread drains and forwards to
  // rmf_arrival_estimator.
  struct PendingFeedback {
    std::size_t waypoint_index;
    double eta_seconds;
  };
  std::optional<PendingFeedback> pending_feedback; // guarded by mutex

  // --- Lane context ---
  std::vector<std::vector<std::size_t>> waypoint_approach_lanes;
  std::vector<std::optional<std::size_t>> waypoint_graph_indices;
  std::atomic<std::size_t> current_target_index{0};

  void invalidate() {
    done.store(true);
    goal_id.reset();
    completion_callback = nullptr;
    failure_callback = nullptr;
    arrival_estimator = nullptr;
    rmf_arrival_estimator = nullptr;
    pending_feedback.reset();
  }
};

} // namespace egb_fleet

#endif // EGB_FLEET__NAVIGATION_SESSION_HPP_
