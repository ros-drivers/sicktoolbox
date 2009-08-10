noinst_PROGRAMS=lms_mean_values
lms_mean_values_SOURCES=main.cc
lms_mean_values_LDADD=-lsicklms-1.0 $(PTHREAD_CFLAGS) $(PTHREAD_LIBS)
lms_mean_values_LDFLAGS=-L$(top_srcdir)/c++/drivers/lms/$(SICK_LMS_SRC_DIR)
AM_CPPFLAGS=-I$(top_srcdir)/c++/drivers/lms -I$(top_srcdir)/c++/drivers/base/src $(PTHREAD_CFLAGS) $(PTHREAD_LIBS) $(all_includes)
