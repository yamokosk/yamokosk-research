#include "common.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{                                                                            
    try {                                                 
        char* conffile = mxArrayToString(prhs[0]);
        char* robotname = mxArrayToString(prhs[1]);
        //Robot robj(conffile, robotname);
        
        mexPrintf("conffile=%s, robotname=%s\n", conffile, robotname);
        Robot robj(conffile, robotname);
        
        // Create link structure
        mxArray *links = mxCreateStructMatrix(robj.get_dof(), 1, NUM_LINK_FIELDS, link_field_names);
        double *ptr = NULL;
        Link *rlinks = robj.links;
        for (int i=1; i <= robj.get_dof(); ++i) 
        {
            //!< Return the joint type.
            mxSetField(links, i, "joint_type", mxCreateDoubleScalar( rlinks[i].get_joint_type() ));
            
            //!< Return theta.
            mxSetField(links, i, "theta", mxCreateDoubleScalar( rlinks[i].get_theta() ));
            
            //!< Return d.
            mxSetField(links, i, "d", mxCreateDoubleScalar( rlinks[i].get_d() ));
            
            //!< Return a.
            mxSetField(links, i, "a", mxCreateDoubleScalar( rlinks[i].get_a() ));
            
            //!< Return alpha.
            mxSetField(links, i, "alpha", mxCreateDoubleScalar( rlinks[i].get_alpha() ));
            
            //!< Return q
            mxSetField(links, i, "q", mxCreateDoubleScalar( rlinks[i].get_q() ));
            
            //!< Return theta_min.
            mxSetField(links, i, "theta_min", mxCreateDoubleScalar( rlinks[i].get_theta_min() ));
            
            //!< Return theta_max.
            mxSetField(links, i, "theta_max", mxCreateDoubleScalar( rlinks[i].get_theta_max() ));
            
            //!< Return joint_offset.
            mxSetField(links, i, "joint_offset", mxCreateDoubleScalar( rlinks[i].get_joint_offset() ));
            
            //!< Return r.
            mxArray *mr = mxCreateDoubleMatrix(1,3,mxREAL); ptr = mxGetPr(mr);
            ColumnVector r = rlinks[i].get_r();
            //mexPrintf("r(%d) = %d\n", i, r.Storage());
            for (int n=0; n < 3; ++n) ptr[n] = r(n+1);
            mxSetField(links, i, "r", mr);
            
            //!< Return p.
            mxArray *mp = mxCreateDoubleMatrix(1,3,mxREAL); ptr = mxGetPr(mp);
            ColumnVector p = rlinks[i].get_p();
            //mexPrintf("p(%d) = %d\n", i, p.Storage());
            for (int n=0; n < 3; ++n) ptr[n] = p(n+1);
            mxSetField(links, i, "p", mp);
            
            //!< Return m.
            mxSetField(links, i, "m", mxCreateDoubleScalar( rlinks[i].get_m() ));
            
            //!< Return Im.
            mxSetField(links, i, "Im", mxCreateDoubleScalar( rlinks[i].get_Im() ));
            
            //!< Return Gr.
            mxSetField(links, i, "Gr", mxCreateDoubleScalar( rlinks[i].get_Gr() ));
            
            //!< Return B.
            mxSetField(links, i, "B", mxCreateDoubleScalar( rlinks[i].get_B() ));
            
            //!< Return Cf.
            mxSetField(links, i, "Cf", mxCreateDoubleScalar( rlinks[i].get_Cf() ));
            
            //!< Return I.
            mxArray *mI = mxCreateDoubleMatrix(3,3,mxREAL);
            Matrix I = rlinks[i].get_I();
            NMMatrixToMxArray(mxGetPr(mI), I, 3, 3);
            //mexPrintf("inertia(%d) = %d\n", i, inertia.Storage());
            mxSetField(links, i, "I", mI);
            
            //!< Return immobile.
            mxSetField(links, i, "immobile", mxCreateLogicalScalar( rlinks[i].get_immobile() ));
        }
        
        // Create robot structure
        LHS_ARG_1 = mxCreateStructMatrix(1, 1, NUM_ROBOT_FIELDS, robot_field_names);
        
        // Set name
        mxSetField(LHS_ARG_1, 0, "name", mxCreateString(robotname) );
        
        // Return DH type
        mxSetField(LHS_ARG_1, 0, "DH", mxCreateLogicalScalar( robj.get_DH() ));
        
        // Return DOF
        mxSetField(LHS_ARG_1, 0, "dof", mxCreateDoubleScalar( (double)robj.get_dof() ));
        
        // Return available DOF
        mxSetField(LHS_ARG_1, 0, "available_dof", mxCreateDoubleScalar( robj.get_available_dof() ));
        
        // Return links
        mxSetField(LHS_ARG_1, 0, "links", links);
        
        // Free allocated stuff
        mxFree( conffile );
        mxFree( robotname );
    
    } catch(Exception) {                                                        
        std::ostringstream msg;                                                
        msg << mexFunctionName() << ":: " << Exception::what();                 
        mexErrMsgTxt(msg.str().c_str());                                        
	} catch (const std::runtime_error& e) {                                     
        std::ostringstream msg;                                                 
        msg << mexFunctionName() << ":: " << e.what();                          
        mexErrMsgTxt(msg.str().c_str());                                        
	} catch (...) {                                                             
        std::ostringstream msg;                                                 
        msg << mexFunctionName() << ":: Unknown failure during operation";      
        mexErrMsgTxt(msg.str().c_str());                                        
	}                                                                           
}
   
