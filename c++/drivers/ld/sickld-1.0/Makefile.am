hh_sources= SickLD.hh \
            SickLDMessage.hh \
            SickLDBufferMonitor.hh \
            SickLDUtility.hh \
	    $(top_srcdir)/c++/drivers/base/src/SickLIDAR.hh \
	    $(top_srcdir)/c++/drivers/base/src/SickMessage.hh \
	    $(top_srcdir)/c++/drivers/base/src/SickBufferMonitor.hh \
	    $(top_srcdir)/c++/drivers/base/src/SickException.hh \
	    $(top_srcdir)/c++/drivers/base/src/SickConfig.hh

cc_sources= SickLD.cc \
            SickLDMessage.cc \
            SickLDBufferMonitor.cc

library_includedir=$(includedir)/sickld-1.0/
library_include_HEADERS=$(hh_sources)

lib_LTLIBRARIES=libsickld-1.0.la
libsickld_1_0_la_SOURCES=$(hh_sources) $(cc_sources)
AM_CPPFLAGS=-I$(top_srcdir)/c++/drivers/base/src