# force automatic escaping of preprocessor definitions
cmake_policy(PUSH)
cmake_policy(SET CMP0005 NEW)

add_definitions(-DDJI_ROBOMASTER_EP_DRIVER__SUPPORTED_ROS_VERSION=1)

cmake_policy(POP)
