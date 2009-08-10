noinst_PROGRAMS=lms2xx_set_variant
lms2xx_set_variant_SOURCES=main.cc
lms2xx_set_variant_LDADD=-lsicklms2xx $(PTHREAD_CFLAGS) $(PTHREAD_LIBS)
lms2xx_set_variant_LDFLAGS=-L$(top_srcdir)/c++/drivers/lms2xx/$(SICK_LMS_2XX_SRC_DIR)
AM_CPPFLAGS=-I$(top_srcdir)/c++/drivers/lms2xx -I$(top_srcdir)/c++/drivers/base/src $(PTHREAD_CFLAGS) $(PTHREAD_LIBS) $(all_includes)
