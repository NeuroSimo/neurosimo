#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "decider_wrapper.h"
#include "log_ipc_server.h"

namespace py = pybind11;

void DeciderWrapper::log(const std::string& message) {
  /* Send via IPC (works from any process). */
  log_ipc_client::send_to_parent(message);
}

PYBIND11_MODULE(cpp_bindings, m) {
  m.def("log", &DeciderWrapper::log, py::arg("message"));
}
