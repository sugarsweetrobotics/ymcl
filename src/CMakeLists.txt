set(srcs ymcl.c laser_likelihood.c laser_raytracing.c ranger.c particle.c resampling.c motion_model.c random.c map.c)

include_directories(${PROJECT_SOURCE_DIR}/include)

set(headers)
foreach(hdr ${hdrs})
  set(headers ${headers}  "${hdr}")
endforeach(hdr)

message(${headers})

add_library(${PROJECT_NAME} SHARED ${srcs} ${headers})
# set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "")
target_link_libraries(${PROJECT_NAME})

if(WIN32)
  add_definitions("/D _YMCL_USE_STATIC")
endif(WIN32)

add_library(${PROJECT_NAME}Static STATIC ${srcs} ${headers})
# set_target_properties(${PROJECT_NAME}Static PROPERTIES PREFIX "")
target_link_libraries(${PROJECT_NAME}Static)
