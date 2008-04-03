/*
mexROBOOP -- A Matlab wrapper of the RoboOp C++ library
Copyright (C) 2008	J.D. Yamokoski

This library is free software; you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as
published by the Free Software Foundation; either version 2.1 of the
License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

-------------------------------------------------------------------------------
Revision_history:

2008/03/10: J.D. Yamokoski
	- Created.
-------------------------------------------------------------------------------
*/

#ifndef ROBOOP_MEX_ERROR_H
#define ROBOOP_MEX_ERROR_H

#define COMPONENT			"RoboopMex"

#define	INVALID_NUM_ARGS	"InvalidArgumentNumber"
#define INVALID_ARG			"InvalidArgument"
#define	NEWMAT_ERROR		"NewmatError"
#define RUNTIME_ERROR		"RuntimeError"

#define ERROR_MSG(id,msg)															\
    mexErrMsgIdAndTxt(COMPONENT ":" id, "%s\n\nError in ==> %s at %d", 				\
		msg, __FILE__, __LINE__)
		
		
#endif