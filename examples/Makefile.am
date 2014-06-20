## Process this file with automake to produce Makefile.in
## ./examples/Makefile.am

AM_CPPFLAGS=-I${top_srcdir}/include/ -DDEBUG_LEVEL=5
AM_LDFLAGS=-L../src
LDADD = ../src/libbbb-eqep.a

noinst_PROGRAMS = test

test_SOURCES = test.cpp
