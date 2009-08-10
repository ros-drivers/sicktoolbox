lib_include_hh= SickLMS2xx.hh \
                SickLMS2xxMessage.hh \
                SickLMS2xxBufferMonitor.hh \
	        $(top_srcdir)/c++/drivers/base/src/SickLIDAR.hh \
	        $(top_srcdir)/c++/drivers/base/src/SickMessage.hh \
	        $(top_srcdir)/c++/drivers/base/src/SickBufferMonitor.hh \
	        $(top_srcdir)/c++/drivers/base/src/SickException.hh

hh_sources= $(lib_include_hh) \
	    $(top_srcdir)/c++/drivers/base/src/SickConfig.hh \
            SickLMS2xxUtility.hh 

cc_sources= SickLMS2xx.cc \
            SickLMS2xxMessage.cc \
            SickLMS2xxBufferMonitor.cc

library_includedir=$(includedir)/sicklms2xx/
library_include_HEADERS=$(lib_include_hh)

lib_LTLIBRARIES=libsicklms2xx.la
libsicklms2xx_la_SOURCES=$(hh_sources) $(cc_sources)
AM_CPPFLAGS=-I$(top_srcdir)/c++/drivers/base/src