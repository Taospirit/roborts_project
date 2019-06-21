project(mvsua_driver)

#uvc_driver
add_library(mvsua_driver
  SHARED
  mvsua_driver.cpp
  )

#请执行installer中的install脚本，或将rules复制到/etc/udev/rules.d/
if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_libraries(mvsua_driver
    PRIVATE
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${CMAKE_CURRENT_SOURCE_DIR}/MVSDK/lib/arm64/libMVSDK.so
    )
else ()
  target_link_libraries(mvsua_driver
    PRIVATE
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${CMAKE_CURRENT_SOURCE_DIR}/MVSDK/lib/x64/libMVSDK.so
    )
endif ()

target_include_directories(mvsua_driver
  PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRECTORIES}
  )

add_library(driver::mvsua_driver ALIAS mvsua_driver)
