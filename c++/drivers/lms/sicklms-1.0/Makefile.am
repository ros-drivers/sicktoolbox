hh_sources= SickLMS.hh \
            SickLMSMessage.hh \
            SickLMSBufferMonitor.hh \
            SickLMSUtility.hh \
	    $(top_srcdir)/c++/drivers/base/src/SickLIDAR.hh \
	    $(top_srcdir)/c++/drivers/base/src/SickMessage.hh \
	    $(top_srcdir)/c++/drivers/base/src/SickBufferMonitor.hh \
	    $(top_srcdir)/c++/drivers/base/src/SickException.hh \
	    $(top_srcdir)/c++/drivers/base/src/SickConfig.hh

cc_sources= SickLMS.cc \
            SickLMSMessage.cc \
            SickLMSBufferMonitor.cc

library_includedir=$(includedir)/sicklms-1.0/
library_include_HEADERS=$(hh_sources)

lib_LTLIBRARIES=libsicklms-1.0.la
libsicklms_1_0_la_SOURCES=$(hh_sources) $(cc_sources)
AM_CPPFLAGS=-I$(top_srcdir)/c++/drivers/base/src