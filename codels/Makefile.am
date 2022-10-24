
lib_LTLIBRARIES = libCT_drone_codels.la

libCT_drone_codels_la_SOURCES  =	CT_drone_c_types.h
libCT_drone_codels_la_SOURCES +=	CT_drone_codels.c
libCT_drone_codels_la_SOURCES +=	CT_drone_track_codels.c

libCT_drone_codels_la_CPPFLAGS =	$(requires_CFLAGS)
libCT_drone_codels_la_LIBADD   =	$(requires_LIBS)
libCT_drone_codels_la_CPPFLAGS+=	$(codels_requires_CFLAGS)
libCT_drone_codels_la_LIBADD  +=	$(codels_requires_LIBS)
libCT_drone_codels_la_LDFLAGS  =	-release $(PACKAGE_VERSION)


# idl  mappings
BUILT_SOURCES=	CT_drone_c_types.h
CLEANFILES=	${BUILT_SOURCES}

CT_drone_c_types.h: ${top_srcdir}/ColorTracker.gen
	${GENOM3}  mappings \
	  -MD -MF .deps/$@.d -MT $@ --signature -l c $< >$@

-include .deps/CT_drone_c_types.h.d
