#####################################################
#
#  FILE: ax_header_linux_serial.m4
#  AUTH: Jason Derenick at Lehigh University
#  CONT: <derenick(at)lehigh(dot)edu>
#  DATE: 20 December 2007
#
#  DESC: A very simple macro for detecting
#        linux/serial.h
#
#####################################################
AC_DEFUN([AX_HEADER_LINUX_SERIAL], [

# Check if we can find linux/serial.h
AC_CHECK_HEADER([linux/serial.h],
		[AC_DEFINE(HAVE_LINUX_SERIAL_H,1,[Define if you have linux/serial.h])],
		)

])dnl AX_HEADER_LINUX_SERIAL
