SUBDIRS=c++
include aminclude.am
MOSTLYCLEANFILES= $(DX_CLEANFILES)
EXTRA_DIST= matlab/config_mex \
            matlab/build_mex \
	    matlab/install_mex \
            matlab/.sed/pathdef.sed \
	    matlab/interfaces/ld/ldmex.cc \
	    matlab/interfaces/ld/sickld.m \
            matlab/interfaces/lms/lmsmex.cc \
	    matlab/interfaces/lms/sicklms.m \
	    matlab/examples/ld/ld_cart.m \
            matlab/examples/ld/ld_stream.m \
	    matlab/examples/lms/lms_cart.m \
	    matlab/examples/lms/lms_stream.m \
	    matlab/examples/lms/lms_variant.m \
	    matlab/misc/info.xml \
            matlab/misc/vader.gif \
	    c++/examples/ld/ld_config/conf/sickld.conf \
	    manuals/sicktoolbox-quickstart.pdf \
	    manuals/sicktoolbox-RS-422.pdf \
	    doxygen.cfg

ld-all: ld-lib ld-examples

lms-all: lms-lib lms-examples

ld-install:
	cd c++/drivers/ld && make install
	cd c++/examples/ld && make install

lms-install:
	cd c++/drivers/lms && make install
	cd c++/drivers/lms && make install

ld-uninstall:
	cd c++/drivers/ld && make uninstall
	cd c++/examples/ld && make uninstall

lms-uninstall:
	cd c++/drivers/lms && make uninstall
	cd c++/examples/lms && make uninstall

ld-clean:
	cd c++/drivers/ld && make clean
	cd c++/examples/ld && make clean

lms-clean:
	cd c++/drivers/lms && make clean
	cd c++/examples/lms && make clean

ld-lib:
	cd c++/drivers/ld && make

lms-lib:
	cd c++/drivers/lms && make

ld-examples:
	cd c++/examples/ld && make

lms-examples:
	cd c++/examples/lms && make
