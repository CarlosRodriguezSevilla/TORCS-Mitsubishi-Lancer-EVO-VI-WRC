##############################################################################
#
#    file                 : Makefile
#    created              : sáb nov 2 10:45:42 CET 2013
#    copyright            : (C) 2002 Stalin Yajamin - Carlos Rodriguez
#
##############################################################################
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
##############################################################################

ROBOT       = bh0152_bh0137_Competicion
MODULE      = ${ROBOT}.so
MODULEDIR   = drivers/${ROBOT}
SOURCES     = ${ROBOT}.cpp

SHIPDIR     = drivers/${ROBOT}
SHIP        = ${ROBOT}.xml pw-evoviwrc.rgb logo.rgb
SHIPSUBDIRS = 

PKGSUBDIRS  = ${SHIPSUBDIRS}
src-robots-bh0152_bh0137_Competicion_PKGFILES = $(shell find * -maxdepth 0 -type f -print)
src-robots-bh0152_bh0137_Competicion_PKGDIR   = ${PACKAGE}-${VERSION}/$(subst ${TORCS_BASE},,$(shell pwd))

include ${MAKE_DEFAULT}
