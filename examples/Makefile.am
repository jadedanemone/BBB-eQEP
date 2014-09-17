## Process this file with automake to produce Makefile.in
## ./examples/Makefile.am

AM_CPPFLAGS=-I${top_srcdir}/include/ -DDEBUG_LEVEL=5
LDADD = ../src/libbbb-eqep.la

noinst_PROGRAMS = test test_multiple

test_SOURCES = test.cpp

test_multiple_SOURCES = test_multiple.cpp
