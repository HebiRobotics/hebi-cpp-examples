file(REMOVE_RECURSE
  "libhebic++-static.pdb"
  "libhebic++-static.a"
)

# Per-language clean rules from dependency scanning.
foreach(lang CXX)
  include(CMakeFiles/hebic++-static.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
