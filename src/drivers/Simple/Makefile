##############################################################################
#
#    file                 : Makefile
#    created              : Do 22. Apr 11:54:37 CEST 2021
#    copyright            : (C) 2002 Jokke Jansen
#
##############################################################################
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
##############################################################################

ROBOT       = Simple
MODULE      = ${ROBOT}.so
MODULEDIR   = drivers/${ROBOT}
SOURCES     = ${ROBOT}.cpp TorcsPomdp.cpp sensors.cpp 

SHIPDIR     = drivers/${ROBOT}
SHIP        = ${ROBOT}.xml pw-306wrc.rgb logo.rgb
SHIPSUBDIRS = 

PKGSUBDIRS  = ${SHIPSUBDIRS}
src-robots-Simple_PKGFILES = $(shell find * -maxdepth 0 -type f -print)
src-robots-Simple_PKGDIR   = ${PACKAGE}-${VERSION}/$(subst ${TORCS_BASE},,$(shell pwd))

include ${MAKE_DEFAULT}
