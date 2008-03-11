#####################################################
#
#  FILE: ax_prog_matlab.m4
#  AUTH: Jason Derenick at Lehigh University
#  CONT: <derenick(at)lehigh(dot)edu>
#  DATE: 20 December 2007
#
#  DESC: A very simple macro for detecting gnuplot
#
#####################################################
AC_DEFUN([AX_PROG_GNUPLOT], [

# Check if we can find gnuplot
AC_CHECK_PROG(have_gnuplot,[gnuplot],yes,no)
if test "$have_gnuplot" = yes; then
  AC_DEFINE(HAVE_GNUPLOT,1,[Define if you have gnuplot])   
else
  AC_MSG_WARN([*** gnuplot not found, some examples will not be built.])
fi

# Register the conditional
AM_CONDITIONAL(HAVE_GNUPLOT,test "$have_gnuplot" = yes)

])dnl AX_PROG_GNUPLOT
