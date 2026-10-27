cmake_minimum_required(VERSION 3.8)
project(oomwoo_cleaning_jobs_lab_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(oomwoo_cleaning_jobs_lab_interfaces
  "srv/StartCleaning.srv"
  "srv/ControlCleaning.srv"
  "srv/GetCleaningStatus.srv"
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
