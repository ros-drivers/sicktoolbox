#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.
  
AC_PREREQ(2.50)
AC_INIT(SickToolbox,1.0,derenick@lehigh.edu)
AC_CONFIG_SRCDIR([c++/drivers/base/src/SickConfig.hh.in])
AC_CONFIG_AUX_DIR(misc)
AM_INIT_AUTOMAKE
AM_CONFIG_HEADER([c++/drivers/base/src/SickConfig.hh])
	
# Set source dirs
AX_SICK_LD_SRC_DIR(sickld-1.0)
AX_SICK_LMS_SRC_DIR(sicklms-1.0)
	
# Checks for programs.
AC_PROG_CXX
AC_PROG_CC
AC_PROG_CPP
AC_PROG_INSTALL
AC_PROG_LN_S
AC_PROG_MAKE_SET
AM_PROG_LIBTOOL
AX_PROG_GNUPLOT
		
# Configure and check for doxygen
DX_HTML_FEATURE(ON)
DX_CHM_FEATURE(OFF)
DX_CHI_FEATURE(OFF)
DX_MAN_FEATURE(ON)
DX_RTF_FEATURE(OFF)
DX_XML_FEATURE(OFF)
DX_PDF_FEATURE(ON)
DX_PS_FEATURE(OFF)
DX_INIT_DOXYGEN(sicktoolbox,doxygen.cfg)			  
		
# Checks for libraries.
ACX_PTHREAD(,[AC_MSG_ERROR([Couldn't find pthread lib!])])
	  
# Checks for header files.
AC_HEADER_STDC
AC_CHECK_HEADERS([arpa/inet.h fcntl.h netinet/in.h string.h sys/ioctl.h sys/socket.h sys/time.h termios.h unistd.h],,[AC_MSG_ERROR([Couldn't find a required header!])])
AX_HEADER_LINUX_SERIAL

# Checks for typedefs, structures, and compiler characteristics.
AC_HEADER_STDBOOL
AC_C_CONST
AC_C_INLINE
AC_C_BIGENDIAN
AC_HEADER_TIME
AC_TYPE_INT16_T
AC_TYPE_UINT16_T
AC_TYPE_UINT32_T
AC_TYPE_UINT8_T

# Checks for library functions.
AC_PROG_GCC_TRADITIONAL
AC_FUNC_SELECT_ARGTYPES
AC_TYPE_SIGNAL
AC_CHECK_FUNCS([gettimeofday memset select socket strcasecmp],,[AC_MSG_ERROR([Couldn't find a required function!])])

AC_CONFIG_FILES([Makefile
                 c++/Makefile
                 c++/examples/Makefile
                 c++/examples/ld/Makefile
                 c++/examples/ld/ld_config/Makefile
                 c++/examples/ld/ld_config/src/Makefile
                 c++/examples/ld/ld_more_config/Makefile
                 c++/examples/ld/ld_more_config/src/Makefile
                 c++/examples/ld/ld_multi_sector/Makefile
                 c++/examples/ld/ld_multi_sector/src/Makefile
                 c++/examples/ld/ld_single_sector/Makefile
                 c++/examples/ld/ld_single_sector/src/Makefile
                 c++/examples/lms/Makefile
                 c++/examples/lms/lms_config/Makefile
                 c++/examples/lms/lms_config/src/Makefile
                 c++/examples/lms/lms_mean_values/Makefile
                 c++/examples/lms/lms_mean_values/src/Makefile
                 c++/examples/lms/lms_partial_scan/Makefile
                 c++/examples/lms/lms_partial_scan/src/Makefile
                 c++/examples/lms/lms_plot_values/Makefile
                 c++/examples/lms/lms_plot_values/src/Makefile
                 c++/examples/lms/lms_real_time_indices/Makefile
                 c++/examples/lms/lms_real_time_indices/src/Makefile
                 c++/examples/lms/lms_set_variant/Makefile
                 c++/examples/lms/lms_set_variant/src/Makefile
		 c++/examples/lms/lms_simple_app/Makefile
                 c++/examples/lms/lms_simple_app/src/Makefile
		 c++/examples/lms/lms_stream_range_and_reflect/Makefile
                 c++/examples/lms/lms_stream_range_and_reflect/src/Makefile		 
		 c++/examples/lms/lms_subrange/Makefile
                 c++/examples/lms/lms_subrange/src/Makefile		 
		 c++/drivers/Makefile
                 c++/drivers/ld/Makefile
                 c++/drivers/ld/sickld-1.0/Makefile
                 c++/drivers/lms/Makefile
                 c++/drivers/lms/sicklms-1.0/Makefile])
		 
AC_OUTPUT
