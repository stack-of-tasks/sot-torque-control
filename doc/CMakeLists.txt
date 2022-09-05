#
# Copyright 2010 CNRS Author: Florent Lamiraux
#

# get the python script install path

if(BUILD_PYTHON_INTERFACE)
  sphinx_setup()
  sphinx_finalize()
  install(FILES pictures/feature.png pictures/task.png
          DESTINATION ${_PKG_CONFIG_DOXYGENDOCDIR}/pictures)
endif(BUILD_PYTHON_INTERFACE)
