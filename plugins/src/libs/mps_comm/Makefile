#*****************************************************************************
#           Makefile Build System for Fawkes: mps_comm Library
#                            -------------------
#   Created on Thu Feb 14 09:30:32 2013
#   Copyright (C) 2013 by Tim Niemueller, AllemaniACs RoboCup Team
#                 2019 by Till Hofmann
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

BASEDIR = ../../..
include $(BASEDIR)/etc/buildsys/config.mk
include $(BUILDSYSDIR)/protobuf.mk
include $(BUILDSYSDIR)/boost.mk
include $(BUILDSYSDIR)/freeopcua.mk

LIBS_libmps_comm = stdc++ m pthread

OBJS_opcua = mps_server.o opc_utils.o

ifeq ($(HAVE_FREEOPCUA),1)
  OBJS_libmps_comm += $(OBJS_opcua)
  CFLAGS  += $(CFLAGS_FREEOPCUA) -DHAVE_FREEOPCUA
  LDFLAGS += $(LDFLAGS_FREEOPCUA)
else
  WARN_TARGETS += warning_freeopcua
endif

OBJS_all = $(OBJS_libmps_comm)
LIBS_all = $(LIBDIR)/libmps_comm.so

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: $(WARN_TARGETS)
warning_freeopcua:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)No OPC-UA support$(TNORMAL) ($(WARNING_FREEOPCUA))"
endif

include $(BUILDSYSDIR)/base.mk
