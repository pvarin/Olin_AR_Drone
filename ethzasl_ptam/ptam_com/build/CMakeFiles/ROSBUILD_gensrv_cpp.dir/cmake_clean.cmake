FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/ptam_com/msg"
  "../src/ptam_com/srv"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/ptam_com/KeyFrame_srv.h"
  "../srv_gen/cpp/include/ptam_com/PointCloud.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
