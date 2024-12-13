#include "rclcpp/rclcpp.hpp"

#include "fpga.h"
#include "NiFpga_mTMS.h"

NiFpga_Session session;
NiFpga_Status status;

bool try_init_fpga() {
  /* Must be called before any other calls. */
  status = NiFpga_Initialize();
  if (NiFpga_IsError(status)) {
    RCLCPP_INFO(rclcpp::get_logger("run_fpga"), "FPGA could not be initialized, exiting.");
    return false;
  }

  /* Opens a session, downloads the bitstream, and runs the FPGA. */
  auto bitfile = std::getenv("BITFILE");
  auto bitfile_directory = std::getenv("BITFILE_DIRECTORY");
  auto bitfile_signature = std::getenv("BITFILE_SIGNATURE");
  auto resource = std::getenv("RESOURCE");

  if (!bitfile || !bitfile_directory) {
    RCLCPP_ERROR(rclcpp::get_logger("run_fpga"),
                 "No BITFILE or BITFILE_DIRECTORY environment variable set.");
    return false;
  }
  if (!bitfile_signature) {
    RCLCPP_ERROR(rclcpp::get_logger("run_fpga"),
                 "No BITFILE_SIGNATURE environment variable set.");
    return false;
  }
  if (!resource) {
    RCLCPP_ERROR(rclcpp::get_logger("run_fpga"),
                 "No RESOURCE environment variable set.");
    return false;
  }

  std::string bitfile_path_str = std::string(bitfile_directory) + "/" + std::string(bitfile);
  std::string bitfile_signature_str = std::string(bitfile_signature);
  std::string resource_str = std::string(resource);

  NiFpga_MergeStatus(&status, NiFpga_Open(
      bitfile_path_str.c_str(),
      bitfile_signature_str.c_str(),
      resource_str.c_str(),
      NiFpga_OpenAttribute_NoRun,
      &session));

  if (NiFpga_IsError(status)) {
    RCLCPP_INFO(rclcpp::get_logger("run_fpga"), "FPGA bitfile could not be loaded (resource: %s), exiting. Status: %d",
      resource_str.c_str(), status);

    return false;
  }

  RCLCPP_INFO(rclcpp::get_logger("run_fpga"), "Initialization successful.");
  return true;
}

void init_fpga() {
  while (true) {
    if (try_init_fpga()) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  /* Sleep for a while longer to ensure that there is enough time for 'Run FPGA' node to start the FPGA program. */
  std::this_thread::sleep_for(std::chrono::seconds(1));
}

bool is_fpga_ok() {
  NiFpga_Status status;
  NiFpga_FpgaViState state;
  uint32_t stateValue;

  status = NiFpga_GetFpgaViState(session, &stateValue);

  if (status != NiFpga_Status_Success) {
      RCLCPP_INFO(rclcpp::get_logger("feedback_monitor_bridge"), "Error getting FPGA VI state: %d", status);
      return false;
  }

  state = (NiFpga_FpgaViState)stateValue;
  if (state != NiFpga_FpgaViState_Running) {
      return false;
  }
  return true;
}

bool close_fpga() {
  RCLCPP_INFO(rclcpp::get_logger("run_fpga"), "Closing FPGA connection.");

  /* must close if we successfully opened */
  NiFpga_MergeStatus(&status, NiFpga_Close(session, 0));

  /* must be called after all other calls */
  NiFpga_MergeStatus(&status, NiFpga_Finalize());

  return true;
}
