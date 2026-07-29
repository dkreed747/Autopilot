#ifndef AUTOPILOT_UMAA_CALLBACKOBSERVER_HPP_
#define AUTOPILOT_UMAA_CALLBACKOBSERVER_HPP_

#include <functional>
#include <utility>

#include "Observer.h"

namespace arlcore::autopilot {

//! \brief Observer adapter forwarding a Subject's notifications to a std::function, so one class
//! can observe several subjects of the same payload type.
template <class T>
class CallbackObserver : public arlcore::Observer<T> {
 public:
  explicit CallbackObserver(std::function<void(const T&)> fn) : fn_(std::move(fn)) {}
  void update(const T& data) override { fn_(data); }

 private:
  std::function<void(const T&)> fn_;
};

}  // namespace arlcore::autopilot
#endif  // AUTOPILOT_UMAA_CALLBACKOBSERVER_HPP_
