#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.
AC_PREREQ([2.69])
AC_INIT([BBB-eQEP], [0.1.3], [james.zapico@gmail.com])
AC_CONFIG_SRCDIR([config.h.in])
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_AUX_DIR(auxdir)
AC_CONFIG_MACRO_DIR([m4])

# Checks for programs.
AC_PROG_CXX
AC_PROG_CC
AC_PROG_INSTALL
AC_PROG_AWK
AC_PROG_MKDIR_P
AM_PROG_AR

AC_LANG_PUSH([C++])

# Checks for libraries.
LT_INIT

# Checks for header files.
AC_CHECK_HEADERS([fcntl.h inttypes.h stdint.h stdlib.h string.h sys/ioctl.h unistd.h sys/mman.h])

# Checks for typedefs, structures, and compiler characteristics.
# AX_CXX_COMPILE_STDCXX_11

AC_CHECK_HEADER_STDBOOL
AC_TYPE_UINT8_T
AC_TYPE_UINT16_T
AC_TYPE_UINT32_T
AC_TYPE_INT8_T
AC_TYPE_INT16_T
AC_TYPE_INT32_T


# Checks for library functions.
AC_FUNC_MMAP
AC_CHECK_FUNCS([munmap strerror getpagesize])

AM_INIT_AUTOMAKE([-Wall])

AC_CHECK_PROGS([DOXYGEN], [doxygen])
if test -z "$DOXYGEN";
   then AC_MSG_WARN([Doxygen not found - continuing without Doxygen support])
fi
AM_CONDITIONAL([HAVE_DOXYGEN], 
[test -n "$DOXYGEN"])AM_COND_IF([HAVE_DOXYGEN], [AC_CONFIG_FILES([docs/Doxyfile])])


AC_CONFIG_FILES([
 Makefile
 src/Makefile
 examples/Makefile
 include/Makefile
 dts/Makefile
 docs/Makefile
])

AC_OUTPUT
