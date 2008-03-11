#####################################################
#
#  FILE: ax_sick_lms_src_dir.m4
#  AUTH: Jason C. Derenick
#  CONT: derenick(at)lehigh(dot)edu
#  DATE: 20 December 2007
#
#  DESC: A macro used for setting the lms source dir
#
#  NOTE: Assumes there is always an input arg
#
#####################################################
AC_DEFUN([AX_SICK_LMS_SRC_DIR], [
SICK_LMS_SRC_DIR=$1
AC_SUBST(SICK_LMS_SRC_DIR)
])dnl AX_SICK_LMS_SRC_DIR
