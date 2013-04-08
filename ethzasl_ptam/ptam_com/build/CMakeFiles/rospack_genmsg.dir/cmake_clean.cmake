FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/ptam_com/msg"
  "../src/ptam_com/srv"
  "CMakeFiles/rospack_genmsg"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/rospack_genmsg.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
