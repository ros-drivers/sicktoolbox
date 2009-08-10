lib_include_hh= SickLMS1xx.hh \
                SickLMS1xxMessage.hh \
                SickLMS1xxBufferMonitor.hh \
	        $(top_srcdir)/c++/drivers/base/src/SickLIDAR.hh \
	        $(top_srcdir)/c++/drivers/base/src/SickMessage.hh \
	        $(top_srcdir)/c++/drivers/base/src/SickBufferMonitor.hh \
	        $(top_srcdir)/c++/drivers/base/src/SickException.hh

hh_sources= $(lib_include_hh) \
	    $(top_srcdir)/c++/drivers/base/src/SickConfig.hh \
            SickLMS1xxUtility.hh 

cc_sources= SickLMS1xx.cc \
            SickLMS1xxMessage.cc \
            SickLMS1xxBufferMonitor.cc

library_includedir=$(includedir)/sicklms1xx/
library_include_HEADERS=$(lib_include_hh)

lib_LTLIBRARIES=libsicklms1xx.la
libsicklms1xx_la_SOURCES=$(hh_sources) $(cc_sources)
AM_CPPFLAGS=-I$(top_srcdir)/c++/drivers/base/src