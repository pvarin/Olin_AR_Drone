FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/ptam_com/msg"
  "../src/ptam_com/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/ptam_com/srv/__init__.py"
  "../src/ptam_com/srv/_KeyFrame_srv.py"
  "../src/ptam_com/srv/_PointCloud.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
