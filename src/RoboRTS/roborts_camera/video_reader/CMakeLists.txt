project(video_reader)

#video_reader for test
add_library(video_reader
  SHARED
  video_reader.cpp
  )

target_link_libraries(video_reader
  PRIVATE
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  )

target_include_directories(video_reader
  PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRECTORIES}
  )

add_library(driver::video_reader ALIAS video_reader)
