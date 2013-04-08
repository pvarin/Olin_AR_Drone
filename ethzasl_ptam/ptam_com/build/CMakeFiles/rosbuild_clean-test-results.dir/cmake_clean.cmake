FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/ptam_com/msg"
  "../src/ptam_com/srv"
  "CMakeFiles/rosbuild_clean-test-results"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/rosbuild_clean-test-results.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
