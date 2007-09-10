#include <xercesc/util/PlatformUtils.hpp>
#include <xercesc/parsers/AbstractDOMParser.hpp>
#include <xercesc/dom/DOMImplementation.hpp>
#include <xercesc/dom/DOMImplementationLS.hpp>
#include <xercesc/dom/DOMImplementationRegistry.hpp>
#include <xercesc/dom/DOMBuilder.hpp>
#include <xercesc/dom/DOMException.hpp>
#include <xercesc/dom/DOMDocument.hpp>
#include <xercesc/dom/DOMNodeList.hpp>
#include <xercesc/dom/DOMError.hpp>
#include <xercesc/dom/DOMLocator.hpp>
#include <xercesc/dom/DOMNamedNodeMap.hpp>
#include <xercesc/dom/DOMAttr.hpp>

#include "planner\sceneparser.h"


static int countChildElements(DOMNode *n, bool printOutEncounteredEles)
{
    DOMNode *child;
    int count = 0;
    if (n) {
        if (n->getNodeType() == DOMNode::ELEMENT_NODE)
		{
            if(printOutEncounteredEles) {
                char *name = XMLString::transcode(n->getNodeName());
                XERCES_STD_QUALIFIER cout <<"----------------------------------------------------------"<<XERCES_STD_QUALIFIER endl;
                XERCES_STD_QUALIFIER cout <<"Encountered Element : "<< name << XERCES_STD_QUALIFIER endl;
                
                XMLString::release(&name);
			
                if(n->hasAttributes()) {
                    // get all the attributes of the node
                    DOMNamedNodeMap *pAttributes = n->getAttributes();
                    int nSize = pAttributes->getLength();
                    XERCES_STD_QUALIFIER cout <<"\tAttributes" << XERCES_STD_QUALIFIER endl;
                    XERCES_STD_QUALIFIER cout <<"\t----------" << XERCES_STD_QUALIFIER endl;
                    for(int i=0;i<nSize;++i) {
                        DOMAttr *pAttributeNode = (DOMAttr*) pAttributes->item(i);
                        // get attribute name
                        char *name = XMLString::transcode(pAttributeNode->getName());
                        
                        XERCES_STD_QUALIFIER cout << "\t" << name << "=";
                        XMLString::release(&name);
                        
                        // get attribute type
                        name = XMLString::transcode(pAttributeNode->getValue());
                        XERCES_STD_QUALIFIER cout << name << XERCES_STD_QUALIFIER endl;
                        XMLString::release(&name);
                    }
                }
            }
			++count;
		}
        for (child = n->getFirstChild(); child != 0; child=child->getNextSibling())
            count += countChildElements(child, printOutEncounteredEles);
    }
    return count;
}

/*ElementFinder::ElementFinder() :
	TAG_SPACE_("space"),
	TAG_BODY_("body"),
	TAG_TRANSFORM_("transform"),
	TAG_GEOMETRY_("geometry"),
	TAG_TRANSLATION_("translation"),
	TAG_ROTATION_("rotation"),
	TAG_PARAMETER_("parameter"),
	ATTR_NAME_("name"),
	ATTR_PARENT_("parent"),
	ATTR_TYPE_("type"),
	ATTR_VALUE_("value"),
	ATTR_MUTABLE_("mutable"),
	xmlDoc_( NULL )
{
	return;
}

xercesc::DOMElement* ElementFinder::getSceneElement()
{
	// We could also have called xml->getFirstChild() and worked
	// with DOMNode objects (DOMDocument is also a DOMNode); but
	// DOMNode only lets us get at the tree using other abstract
	// DOMNodes.  In turn, that would require us to walk the tree
	// and query each node for its name before we do anything with
	// the data.

	// <config/> element
	xercesc::DOMElement* result = xmlDoc_->getDocumentElement();
	return( result );	
}

xercesc::DOMElement* ElementFinder::getElement( const XMLCh* name )
{
	xercesc::DOMElement* result = NULL;
	xercesc::DOMNodeList* list = xmlDoc_->getElementsByTagName( name );
	xercesc::DOMNode* node = list->item( 0 );

	if( xercesc::DOMNode::ELEMENT_NODE == node->getNodeType() ){
		result = dynamic_cast< xercesc::DOMElement* >( node );
	}

	return( result );
}*/



XMLSceneData::XMLSceneData( const std::string& fromFile ) :
	xmlFile_( fromFile ),
	parser_(NULL)
{
	XMLPlatformUtils::Initialize();

	// DOM parser options
	bool	doNamespaces	= false;
	bool	doSchema		= false;
	bool	schemaFullChecking = false;
	bool	doList			= false;
	bool	errorOccurred	= false;
	bool	recognizeNEL	= false;
	bool	printOutEncounteredEles = false;

	// Instantiate the DOM parser.
	static const XMLCh gLS[] = { chLatin_L, chLatin_S, chNull };
	DOMImplementation *impl = DOMImplementationRegistry::getDOMImplementation(gLS);
	parser_ = ((DOMImplementationLS*)impl)->createDOMBuilder(DOMImplementationLS::MODE_SYNCHRONOUS, 0);

	parser_->setFeature(XMLUni::fgDOMNamespaces, doNamespaces);
	parser_->setFeature(XMLUni::fgXercesSchema, doSchema);
	parser_->setFeature(XMLUni::fgXercesSchemaFullChecking, schemaFullChecking);

    // enable datatype normalization - default is off
	//parser_->setFeature(XMLUni::fgDOMDatatypeNormalization, true);

	// And create our error handler and install it
	//DOMCountErrorHandler errorHandler;
	//parser_->setErrorHandler(&errorHandler);
}

XMLSceneData::~XMLSceneData()
{
	//  Delete the parser itself.  Must be done prior to calling Terminate, below.
    if (parser_) parser_->release();

	// And call the termination method
	XMLPlatformUtils::Terminate();
}

void XMLSceneData::load()	throw( std::runtime_error )
{
	//  Get the starting time and kick off the parse of the indicated
	//  file. Catch any exceptions that might propogate out of it.
    //unsigned long duration;
	
	//reset error count first
	//errorHandler.resetErrors();

	DOMDocument *doc = NULL;
	unsigned long duration;

	try {
		// reset document pool
		parser_->resetDocumentPool();

		const unsigned long startMillis = XMLPlatformUtils::getCurrentMillis();
		doc = parser_->parseURI(xmlFile_.c_str());
		const unsigned long endMillis = XMLPlatformUtils::getCurrentMillis();
		duration = endMillis - startMillis;
	} catch (const XMLException& toCatch) {
		std::cerr << "\nError during parsing: '" << xmlFile_ << "'\n"
			<< "Exception message is:  \n"
			<< StrX(toCatch.getMessage()) << "\n" << std::endl;
		//errorOccurred = true;
		//continue;
		return;
	} catch (const DOMException& toCatch) {
		const unsigned int maxChars = 2047;
		XMLCh errText[maxChars + 1];
		XERCES_STD_QUALIFIER cerr << "\nDOM Error during parsing: '" << xmlFile_ << "'\n"
			<< "DOMException code is:  " << toCatch.code << XERCES_STD_QUALIFIER endl;
		
		if (DOMImplementation::loadDOMExceptionMsg(toCatch.code, errText, maxChars))
			XERCES_STD_QUALIFIER cerr << "Message is: " << StrX(errText) << XERCES_STD_QUALIFIER endl;

		//errorOccurred = true;
		//continue;
		return;
	} catch (...) {
		XERCES_STD_QUALIFIER cerr << "\nUnexpected exception during parsing: '" << xmlFile_ << "'\n";
		//errorOccurred = true;
		//continue;
		return;
	}

	int elementCount = 0;

	if (doc) {
		//XMLCh xa[] = {chAsterisk, chNull};
		//elementCount = doc->getElementsByTagName(xa)->getLength());
		elementCount = countChildElements((DOMNode*)doc->getDocumentElement(), true);
	}

	//finder_.setDocument( doc );
	std::cout << xmlFile_ << ": " << duration << " ms" << std::endl;
	std::cout << "	Num elements: " << elementCount << std::endl;

	return;
}


/*void XMLSceneData::handleElement( const xercesc::DOMElement* element )
{
	//StringManager sm;

	if( xercesc::XMLString::equals( finder_.TAG_SPACE_.asXMLString() , element->getTagName() ) ) {
		// SPACE TAG PROCESSING
		std::cout << "SPACE TAG: FOUND" << std::endl;

		// get user, password attrs
		// this will call functions on the underlying DOMAttr node for us...
		//const char* loginUser = sm.convert( element->getAttribute( finder_.ATTR_USER_.asXMLString() ) );
		//const char* userPassword = sm.convert( element->getAttribute( finder_.ATTR_PASSWORD_.asXMLString() ) );

		//setLoginUser( loginUser );
		//setLoginPassword( userPassword );
		
	} else if( xercesc::XMLString::equals( finder_.TAG_BODY_.asXMLString() , element->getTagName() ) ) {
		// BODY TAG PROCESSING
		std::cout << "BODY TAG: FOUND" << std::endl;
		// get the "lastupdate" attr
		// this will call functions on the underlying DOMAttr node for us...
		//DualString lastUpdate( element->getAttribute( finder_.ATTR_LASTUPDATE_.asXMLString() ) );
		//setLastUpdate( lastUpdate.asCString() );

	} else if( xercesc::XMLString::equals( finder_.TAG_TRANSFORM_.asXMLString() , element->getTagName() ) ){
		// get a list of <report> elements
		//xercesc::DOMNodeList* reportNodes = element->getElementsByTagName( finder_.TAG_REPORT_.asXMLString() );
		//const XMLSize_t reportCount = reportNodes->getLength();

		//for( XMLSize_t reportIndex = 0; reportIndex < reportCount; ++reportIndex ){
		//	xercesc::DOMNode* reportNode = reportNodes->item( reportIndex );
			// thanks to getElementsByTagName(), we already know these are
			// element attributes
		//	xercesc::DOMElement* reportElement = dynamic_cast< xercesc::DOMElement* >( reportNode );
		//	const char* reportName = sm.convert( reportElement->getAttribute( finder_.ATTR_NAME_.asXMLString() ) );
		//	addReport( reportName );
		//}
		// TRANSFORM TAG PROCESSING
		std::cout << "TRANSFORM TAG: FOUND" << std::endl;

	} else if( xercesc::XMLString::equals( finder_.TAG_GEOMETRY_.asXMLString() , element->getTagName() ) ){
		// GEOMETRY TAG PROCESSING
		std::cout << "GEOM TAG: FOUND" << std::endl;
	
	} else if( xercesc::XMLString::equals( finder_.TAG_TRANSLATION_.asXMLString() , element->getTagName() ) ){
		// TRANSLATION TAG PROCESSING
		std::cout << "	TRANSLATION TAG: FOUND" << std::endl;

	} else if( xercesc::XMLString::equals( finder_.TAG_ROTATION_.asXMLString() , element->getTagName() ) ){
		// ROTATION TAG PROCESSING
		std::cout << "	ROTATION TAG: FOUND" << std::endl;

	} else if( xercesc::XMLString::equals( finder_.TAG_PARAMETER_.asXMLString() , element->getTagName() ) ){
		// PARAMETER TAG PROCESSING
		std::cout << "	PARAMETER TAG: FOUND" << std::endl;

	} else {
		std::ostringstream buf;
		buf << "Unexpected tag: <" << DualString( element->getTagName() ) << "/>" << std::flush;
		throw( std::runtime_error( buf.str() ) );
	}
}*/

std::ostream& XMLSceneData::print( std::ostream& s ) const 
{
	/*s << "[ConfigData: "
		<< "lastUpdate=\"" << getLastUpdate() << "\"/"
		<< "user=\"" << getLoginUser() << "\"/"
		<< "pass=\"" << getLoginPassword() << "\"/"
		<< "reportCount=" << getReportCount()
		<< "]"
		<< std::flush;*/
	return( s );
}
