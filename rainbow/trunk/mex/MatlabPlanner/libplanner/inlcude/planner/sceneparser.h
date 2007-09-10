#ifndef SCENE_FILE_READER_HEADER
#define SCENE_FILE_READER_HEADER

//#include "planner\helper-classes.h"

#include <xercesc/dom/DOMErrorHandler.hpp>
#include <xercesc/dom/DOMBuilder.hpp>
#include <xercesc/util/XMLString.hpp>

#include<string>
#include<iostream>
#include<sstream>
#include<stdexcept>
#include<list>

XERCES_CPP_NAMESPACE_USE

// ---------------------------------------------------------------------------
//  This is a simple class that lets us do easy (though not terribly efficient)
//  trancoding of XMLCh data to local code page for display.
// ---------------------------------------------------------------------------
class StrX
{
public :
    // -----------------------------------------------------------------------
    //  Constructors and Destructor
    // -----------------------------------------------------------------------
    StrX(const XMLCh* const toTranscode)
    {
        // Call the private transcoding method
        fLocalForm = XMLString::transcode(toTranscode);
    }
    ~StrX() { XMLString::release(&fLocalForm); }

    // -----------------------------------------------------------------------
    //  Getter methods
    // -----------------------------------------------------------------------
    const char* localForm() const { return fLocalForm; }

private :
    // -----------------------------------------------------------------------
    //  Private data members
    //
    //  fLocalForm
    //      This is the local code page form of the string.
    // -----------------------------------------------------------------------
    char*   fLocalForm;
};

inline XERCES_STD_QUALIFIER ostream& operator<<(XERCES_STD_QUALIFIER ostream& target, const StrX& toDump)
{
    target << toDump.localForm();
    return target;
}


// ---------------------------------------------------------------------------
//  This is a simple class that lets us do easy (though not terribly efficient)
//  trancoding of XMLCh data to local code page for display.
// ---------------------------------------------------------------------------
/*class ElementFinder 
{
public:
	ElementFinder();
	xercesc::DOMElement* getSceneElement();
	xercesc::DOMElement* getBodyElement() {return( getElement( TAG_BODY_.asXMLString() ) );}
	xercesc::DOMElement* getGeometryElement() {return( getElement( TAG_GEOMETRY_.asXMLString() ) );}
	xercesc::DOMElement* getTransformElement() {return( getElement( TAG_TRANSFORM_.asXMLString() ) );}
	xercesc::DOMElement* getElement( const XMLCh* name );
	void setDocument( xercesc::DOMDocument* const doc ){xmlDoc_ = doc;}

	const DualString TAG_SPACE_;
	const DualString TAG_BODY_;
	const DualString TAG_TRANSFORM_;
	const DualString TAG_GEOMETRY_;
	const DualString TAG_TRANSLATION_;
	const DualString TAG_ROTATION_;
	const DualString TAG_PARAMETER_;

	const DualString ATTR_NAME_;
	const DualString ATTR_PARENT_;
	const DualString ATTR_TYPE_;
	const DualString ATTR_VALUE_;
	const DualString ATTR_MUTABLE_;

private:
	xercesc::DOMDocument* xmlDoc_;
};*/


class XMLSceneData 
{
public:
	//typedef std::list< std::string > ReportList;
	//typedef ReportList::iterator ReportListIterator;
	//typedef ReportListIterator iterator;

	XMLSceneData( const std::string& fromFile );
	~XMLSceneData();

	void load() throw( std::runtime_error );
	std::ostream& print( std::ostream& s ) const;

	/*const std::string& getLastUpdate() const throw() {return( lastUpdate_ );} 
	void setLastUpdate( const std::string& in ){lastUpdate_ = in;} 
	const std::string& getLoginUser() const throw() {return( loginUser_ );} 
	void setLoginUser( const std::string& in ){loginUser_ = in;}
	const std::string& getLoginPassword() const throw() {return( loginPassword_ );} 
	void setLoginPassword( const std::string& in ){loginPassword_ = in;}
	int getReportCount() const throw() {return( reports_.size() );}
	void addReport( const std::string& report ){reports_.push_back( report );}*/

	//iterator begin() {return( reports_.begin() );}
	//iterator end() {return( reports_.end() );}

private:
	//void handleElement( const DOMElement* element );

	std::string		xmlFile_;
	//std::string lastUpdate_;
	//std::string loginUser_;
	//std::string loginPassword_;
	//ReportList reports_;
	//ElementFinder	finder_;
	DOMBuilder*		parser_;
};




#endif