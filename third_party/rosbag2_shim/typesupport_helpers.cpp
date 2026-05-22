// Minimal rosbag2_cpp typesupport_helpers shim. Implements the upstream API
// using ament_index_cpp + rcpputils — no rosbag2 dependency required.

#include "rosbag2_cpp/typesupport_helpers.hpp"

#include <sstream>
#include <stdexcept>

#include "ament_index_cpp/get_package_prefix.hpp"
#include "rcpputils/shared_library.hpp"
#include "rosidl_runtime_cpp/message_type_support_decl.hpp"

namespace rosbag2_cpp {

std::string
get_typesupport_library_path(const std::string &package_name,
                             const std::string &typesupport_identifier) {
#ifdef _WIN32
  const char *filename_prefix = "";
  const char *filename_extension = ".dll";
  const char *dynamic_library_folder = "/bin/";
#elif __APPLE__
  const char *filename_prefix = "lib";
  const char *filename_extension = ".dylib";
  const char *dynamic_library_folder = "/lib/";
#else
  const char *filename_prefix = "lib";
  const char *filename_extension = ".so";
  const char *dynamic_library_folder = "/lib/";
#endif

  const std::string package_prefix =
      ament_index_cpp::get_package_prefix(package_name);
  return package_prefix + dynamic_library_folder + filename_prefix +
         package_name + "__" + typesupport_identifier + filename_extension;
}

namespace {
std::tuple<std::string, std::string>
extract_package_and_type(const std::string &full_type) {
  // "pkg/Msg" or "pkg/msg/Msg" — package is the first segment.
  const auto slash = full_type.find('/');
  if (slash == std::string::npos) {
    throw std::runtime_error("Invalid type: " + full_type);
  }
  const std::string pkg = full_type.substr(0, slash);
  std::string rest = full_type.substr(slash + 1);
  // Handle the "pkg/msg/Msg" form: skip the middle "msg" segment.
  const auto next_slash = rest.find('/');
  if (next_slash != std::string::npos) {
    rest = rest.substr(next_slash + 1);
  }
  return {pkg, rest};
}
} // namespace

std::shared_ptr<rcpputils::SharedLibrary>
get_typesupport_library(const std::string &type,
                        const std::string &typesupport_identifier) {
  const auto [pkg, _msg] = extract_package_and_type(type);
  const std::string library_path =
      get_typesupport_library_path(pkg, typesupport_identifier);
  return std::make_shared<rcpputils::SharedLibrary>(library_path);
}

const rosidl_message_type_support_t *
get_typesupport_handle(const std::string &type,
                       const std::string &typesupport_identifier,
                       std::shared_ptr<rcpputils::SharedLibrary> library) {
  const auto [pkg, msg] = extract_package_and_type(type);
  const std::string symbol_name =
      std::string{"rosidl_typesupport_cpp__get_message_type_support_handle__"} +
      pkg + "__msg__" + msg;
  using GetTypeSupport = const rosidl_message_type_support_t *(*)();
  auto fn = reinterpret_cast<GetTypeSupport>(library->get_symbol(symbol_name));
  return fn();
}

} // namespace rosbag2_cpp
