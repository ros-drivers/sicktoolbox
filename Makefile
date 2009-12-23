all: installed

TARBALL = build/sicktoolbox-1.0.tar.gz
TARBALL_URL = http://pr.willowgarage.com/downloads/sicktoolbox-1.0.tar.gz
SOURCE_DIR = build/sicktoolbox-1.0
#MD5SUM_FILE = sicktoolbox-1.0.tar.gz.md5sum
UNPACK_CMD = tar xzf
TARBALL_PATCH = stdlib_include.patch

include $(shell rospack find mk)/download_unpack_build.mk

installed: wiped $(SOURCE_DIR)/unpacked
	cd $(SOURCE_DIR) && ./configure --prefix=`rospack find sicktoolbox`/sicktoolbox
	cd $(SOURCE_DIR) && make && make install
	touch installed

wiped: Makefile
	make wipe
	touch wiped

clean:
	-rm -rf sicktoolbox-1.0 sicktoolbox
	rm -f *~ installed

.PHONY : clean wipe

wipe: clean
	rm -f sicktoolbox-1.0.tar.gz
