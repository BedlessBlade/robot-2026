#include "auto/TaskList.h"

TaskList::TaskList(std::vector<std::shared_ptr<Task>> tasks) : m_tasks{tasks} {}

void TaskList::Start(double t) {
  if (m_tasks.size() > 0) {
    m_tasks[0]->Start(t);
  }
}

void TaskList::Update(double t) {
  if (IsDone()) {
    return;
  }

  if (!m_tasks[m_taskIndex]->IsDone()) {
    m_tasks[m_taskIndex]->Update(t);
  } else {
    m_tasks[m_taskIndex]->Stop();
    m_taskIndex++;

    if (!IsDone()) {
      m_tasks[m_taskIndex]->Start(t);
    }
  }
}

void TaskList::Stop() {
  if (!IsDone()) {
    m_tasks[m_taskIndex]->Stop();
    m_done = true;
  }
}

bool TaskList::IsDone() const {
  return (m_taskIndex >= m_tasks.size()) || m_done ||
         (m_taskIndex == m_tasks.size() - 1 && m_tasks[m_taskIndex]->IsDone());
}
