XEN_ROOT ?= ../..
include $(XEN_ROOT)/tools/Rules.mk

MAKE=make
INSTALL=install -c

prefix=$(PREFIX)
bindir=$(prefix)/bin
mandir=$(prefix)/share/man
datadir=$(prefix)/share/qemu
docdir=$(prefix)/share/doc/qemu

CC=gcc-3.4
HOST_CC=gcc
AR=ar
STRIP=strip -s -R .comment -R .note
OS_CFLAGS=
OS_LDFLAGS=
ARCH_CFLAGS=-m32
ARCH_LDFLAGS=-m32
CFLAGS= -Wall -O2 -g -fno-strict-aliasing
LDFLAGS= -g
EXESUF=
AIOLIBS=-lrt -lpthread
ARCH=i386
CONFIG_GDBSTUB=yes
CONFIG_SLIRP=yes
CONFIG_OSS=yes
CONFIG_VNC_TLS=yes
CONFIG_VNC_TLS_CFLAGS= 
CONFIG_VNC_TLS_LIBS=-lgnutls  
VERSION=0.9.1
SRC_PATH=/u/iwj/work/qemu-iwj.git

BUILD_DOCS=yes
CONFIG_SDL=yes
SDL_LIBS=-L/usr/lib -lSDL
SDL_CFLAGS=-I/usr/include/SDL -D_GNU_SOURCE=1 -D_REENTRANT
CONFIG_CURSES=yes
CURSES_LIBS=-lcurses
TOOLS=qemu-img$(EXESUF) 
