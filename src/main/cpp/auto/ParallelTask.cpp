#include "auto/ParallelTask.h"

ParallelTask::ParallelTask(std::vector<std::shared_ptr<Task>> tasks)
    : m_tasks{tasks} {}

void ParallelTask::Start(double t) {
  for (auto &task : m_tasks) {
    task->Start(t);
  }
}

void ParallelTask::Update(double t) {
  for (auto &task : m_tasks) {
    task->Update(t);
  }
}

void ParallelTask::Stop() {
  for (auto &task : m_tasks) {
    task->Stop();
  }
}

bool ParallelTask::IsDone() const {
  for (auto &task : m_tasks) {
    if (!task->IsDone()) {
      return false;
    }
  }

  return true;
}
