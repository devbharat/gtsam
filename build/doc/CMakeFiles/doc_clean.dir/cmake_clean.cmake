FILE(REMOVE_RECURSE
  "CMakeFiles/doc_clean"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/doc_clean.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
