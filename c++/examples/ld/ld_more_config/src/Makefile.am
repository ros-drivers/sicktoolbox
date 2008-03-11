noinst_PROGRAMS=ld_more_config
ld_more_config_SOURCES=main.cc
ld_more_config_LDADD=-lsickld-1.0 $(PTHREAD_CFLAGS) $(PTHREAD_LIBS)
ld_more_config_LDFLAGS=-L$(top_srcdir)/c++/drivers/ld/$(SICK_LD_SRC_DIR)
AM_CPPFLAGS=-I$(top_srcdir)/c++/drivers/ld -I$(top_srcdir)/c++/drivers/base/src $(PTHREAD_CFLAGS) $(PTHREAD_LIBS) $(all_includes)
