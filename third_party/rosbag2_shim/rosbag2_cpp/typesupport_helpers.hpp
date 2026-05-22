// Minimal rosbag2_cpp/typesupport_helpers.hpp shim.
// Matches the API domain_bridge consumes; implementation lives in
// typesupport_helpers.cpp and uses ament_index_cpp + rcpputils.

#ifndef ROSBAG2_CPP__TYPESUPPORT_HELPERS_HPP_
#define ROSBAG2_CPP__TYPESUPPORT_HELPERS_HPP_

#include <memory>
#include <string>
#include <tuple>

#include "rcpputils/shared_library.hpp"
#include "rosidl_runtime_cpp/message_type_support_decl.hpp"

namespace rosbag2_cpp {

std::shared_ptr<rcpputils::SharedLibrary>
get_typesupport_library(const std::string &type,
                        const std::string &typesupport_identifier);

const rosidl_message_type_support_t *
get_typesupport_handle(const std::string &type,
                       const std::string &typesupport_identifier,
                       std::shared_ptr<rcpputils::SharedLibrary> library);

std::string
get_typesupport_library_path(const std::string &package_name,
                             const std::string &typesupport_identifier);

} // namespace rosbag2_cpp

#endif // ROSBAG2_CPP__TYPESUPPORT_HELPERS_HPP_
