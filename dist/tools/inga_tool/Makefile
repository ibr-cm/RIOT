PKG_NAME=inga_tool
PKG_URL=https://github.com/ibr-cm/inga_tool
PKG_VERSION=abb6f8839c258b50d7ce9168eda98ae1bcfd9f23
PKG_LICENSE=BSD-3-Clause
## PKG_BUILDDIR=$(CURDIR)/bin

PKG_SOURCE_DIR = $(CURDIR)/bin
PKG_BUILD_OUT_OF_SOURCE = 0

include $(RIOTBASE)/pkg/pkg.mk

all:
# Start inga_tool build in a clean environment, so variables set by RIOT's build process
# for cross compiling a specific target platform are reset and inga_tool can
# be built cleanly for the native platform.
	env -i PATH="$(PATH)" TERM="$(TERM)" "$(MAKE)" -C "$(PKG_SOURCE_DIR)"
	mv $(PKG_SOURCE_DIR)/inga_tool .