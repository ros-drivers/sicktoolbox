bin_PROGRAMS=ld_config
ld_config_SOURCES=main.cc ConfigFile.cpp ConfigFile.h
ld_config_LDADD=-lsickld-1.0 $(PTHREAD_CFLAGS) $(PTHREAD_LIBS)
ld_config_LDFLAGS=-L$(top_srcdir)/c++/drivers/ld/$(SICK_LD_SRC_DIR)
AM_CPPFLAGS=-I$(top_srcdir)/c++/drivers/ld -I$(top_srcdir)/c++/drivers/ld/src $(PTHREAD_CFLAGS) $(PTHREAD_LIBS) $(all_includes)
