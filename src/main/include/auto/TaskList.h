#pragma once

#include <memory>
#include <vector>

#include "Task.h"

// Runs all of the tasks specified sequentially
class TaskList : public Task {
public:
  TaskList(std::vector<std::shared_ptr<Task>> tasks = {});

  void Start(double t) override;
  void Update(double t) override;
  void Stop() override;

  bool IsDone() const override;

protected:
  std::vector<std::shared_ptr<Task>> m_tasks;

private:
  int m_taskIndex = 0;
  bool m_done = false;
};
