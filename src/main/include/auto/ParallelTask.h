#pragma once

#include <memory>
#include <vector>

#include "Task.h"

// Runs all of the specified tasks in parallel
class ParallelTask : public Task {
public:
  ParallelTask(std::vector<std::shared_ptr<Task>> tasks = {});

  void Start(double t) override;
  void Update(double t) override;
  void Stop() override;

  bool IsDone() const override;

private:
  std::vector<std::shared_ptr<Task>> m_tasks;
};
