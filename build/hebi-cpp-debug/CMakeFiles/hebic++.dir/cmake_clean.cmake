file(REMOVE_RECURSE
  "libhebic++.pdb"
  "libhebic++.so.2.0.1"
  "libhebic++.so"
)

# Per-language clean rules from dependency scanning.
foreach(lang CXX)
  include(CMakeFiles/hebic++.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
