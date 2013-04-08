FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/ptam_com/msg"
  "../src/ptam_com/srv"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/ptam_com/msg/__init__.py"
  "../src/ptam_com/msg/_KeyFrame_msg.py"
  "../src/ptam_com/msg/_ptam_info.py"
  "../src/ptam_com/msg/_Vector3Array.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
