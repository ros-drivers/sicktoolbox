noinst_PROGRAMS=lms1xx_simple_app
lms1xx_simple_app_SOURCES=main.cc
lms1xx_simple_app_LDADD=-lsicklms1xx $(PTHREAD_CFLAGS) $(PTHREAD_LIBS)
lms1xx_simple_app_LDFLAGS=-L$(top_srcdir)/c++/drivers/lms1xx/$(SICK_LMS_1XX_SRC_DIR)
AM_CPPFLAGS=-I$(top_srcdir)/c++/drivers/lms1xx -I$(top_srcdir)/c++/drivers/base/src $(PTHREAD_CFLAGS) $(PTHREAD_LIBS) $(all_includes)
