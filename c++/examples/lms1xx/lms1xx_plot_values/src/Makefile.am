if HAVE_GNUPLOT

noinst_PROGRAMS=lms1xx_plot_values
lms1xx_plot_values_SOURCES=main.cc gnuplot_i.cc gnuplot_i.hpp
lms1xx_plot_values_LDADD=-lsicklms1xx $(PTHREAD_CFLAGS) $(PTHREAD_LIBS)
lms1xx_plot_values_LDFLAGS=-L$(top_srcdir)/c++/drivers/lms1xx/$(SICK_LMS_1XX_SRC_DIR)
AM_CPPFLAGS=-I$(top_srcdir)/c++/drivers/lms1xx -I$(top_srcdir)/c++/drivers/base/src $(PTHREAD_CFLAGS) $(PTHREAD_LIBS) $(all_includes)

endif