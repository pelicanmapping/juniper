SET(TARGET_H
  QSplat
  qsplat_colorquant.h
  qsplat_normquant.h
  qsplat_util.h
  qsplat_spherequant.h
)
SET(TARGET_SRC 
  QSplat.cpp
  qsplat_colorquant.cpp
  qsplat_normquant.cpp
  qsplat_spherequant.cpp
)
SETUP_PLUGIN(qs)
SET(LIB_PUBLIC_HEADERS QSplat)
