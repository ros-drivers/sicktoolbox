if HAVE_GNUPLOT

noinst_PROGRAMS=lms2xx_plot_values
lms2xx_plot_values_SOURCES=main.cc gnuplot_i.cc gnuplot_i.hpp
lms2xx_plot_values_LDADD=-lsicklms2xx $(PTHREAD_CFLAGS) $(PTHREAD_LIBS)
lms2xx_plot_values_LDFLAGS=-L$(top_srcdir)/c++/drivers/lms2xx/$(SICK_LMS_2XX_SRC_DIR)
AM_CPPFLAGS=-I$(top_srcdir)/c++/drivers/lms2xx -I$(top_srcdir)/c++/drivers/base/src $(PTHREAD_CFLAGS) $(PTHREAD_LIBS) $(all_includes)

endif