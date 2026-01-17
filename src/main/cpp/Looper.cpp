#include "Looper.h"

// This default contructor allows us to have an instance of Looper in another
// class without using a pointer.
Looper::Looper() { m_running = false; }

Looper::Looper(std::function<void()> update, double hz) {
  // The notifier is initialized using a smart pointer so that it will
  // automatically be freed and we aren't forced to create a notifier instance
  // when using the default constructor.
  m_notifier = std::make_unique<frc::Notifier>(update);
  m_running = true;

  // Divide 1 by the hz to get the target dt
  m_notifier->StartPeriodic(units::second_t{1 / hz});
}

bool Looper::IsRunning() const { return m_running; }

void Looper::Stop() {
  // Only call stop if the non-default constructor swas used. Otherwise, the
  // notifier won't have been initialized.
  if (m_running) {
    m_notifier->Stop();
    m_running = false;
  }
}
