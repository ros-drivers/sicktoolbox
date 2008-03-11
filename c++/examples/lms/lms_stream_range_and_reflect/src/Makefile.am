noinst_PROGRAMS=lms_stream_range_and_reflect
lms_stream_range_and_reflect_SOURCES=main.cc
lms_stream_range_and_reflect_LDADD=-lsicklms-1.0 $(PTHREAD_CFLAGS) $(PTHREAD_LIBS)
lms_stream_range_and_reflect_LDFLAGS=-L$(top_srcdir)/c++/drivers/lms/$(SICK_LMS_SRC_DIR)
AM_CPPFLAGS=-I$(top_srcdir)/c++/drivers/lms -I$(top_srcdir)/c++/drivers/base/src $(PTHREAD_CFLAGS) $(PTHREAD_LIBS) $(all_includes)
