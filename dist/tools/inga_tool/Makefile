PKG_NAME=inga_tool
PKG_URL=/home/pi/git/inga_tool
PKG_VERSION=8bc91973b4c1f77145a9596ae6f6abb501b169d6
PKG_LICENSE=BSD-3-Clause
PKG_BUILDDIR=$(CURDIR)/bin

include $(RIOTBASE)/pkg/pkg.mk

all:
# Start edbg build in a clean environment, so variables set by RIOT's build process
# for cross compiling a specific target platform are reset and edbg can
# be built cleanly for the native platform.
	env -i PATH="$(PATH)" TERM="$(TERM)" "$(MAKE)" -C "$(PKG_BUILDDIR)"
	mv $(PKG_BUILDDIR)/inga_tool .