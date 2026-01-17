#pragma once

#include <frc/Notifier.h>
#include <functional>
#include <memory>

// The Looper class is responsible for updating the robot's state while the code
// is running. There is one main instance of the Looper in the Robot class
// which is where all the sub-systems are updated.
class Looper {
public:
  Looper();
  Looper(std::function<void()> update, double hz = 200);

  bool IsRunning() const;
  void Stop();

private:
  // The notifier will run the update callback at a fixed interval after it is
  // started.
  std::unique_ptr<frc::Notifier> m_notifier;
  bool m_running;
};
