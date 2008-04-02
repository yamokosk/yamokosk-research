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