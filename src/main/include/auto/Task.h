#pragma once

class Task {
public:
  // Intitializes the task with a starting time
  virtual void Start(double t){};

  // Update the task witht the timestamp
  virtual void Update(double t){};

  // Only call this to stop the task early
  virtual void Stop(){};

  virtual bool IsDone() const { return false; };
};
