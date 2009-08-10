#####################################################
#
#  FILE: ax_sick_lms_1xx_src_dir.m4
#  AUTH: Jason C. Derenick
#  CONT: jasonder(at)seas(dot)upenn(dot)edu
#  DATE: 10 August 2009
#
#  DESC: A macro for setting the lms 2xx source dir
#
#  NOTE: Assumes there is always an input arg
#
#####################################################
AC_DEFUN([AX_SICK_LMS_1XX_SRC_DIR], [
SICK_LMS_1XX_SRC_DIR=$1
AC_SUBST(SICK_LMS_1XX_SRC_DIR)
])dnl AX_SICK_LMS_1XX_SRC_DIR
