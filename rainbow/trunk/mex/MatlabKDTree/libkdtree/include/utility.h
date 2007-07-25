#ifndef SPATIAL_SEARCH_UTILITY_HEADER_FILE
#define SPATIAL_SEARCH_UTILITY_HEADER_FILE

#include <string>
#include <map>

enum StringValue { evNotDefined,
	               evInit,
		           evSearch,
			       evReset,
				   evEnd };

std::map<std::string, StringValue> g_MapStringValues;

void InitializeStringValueMap()
{
	g_MapStringValues["Init"] = evInit;
	g_MapStringValues["Search"] = evSearch;
	g_MapStringValues["Reset"] = evReset;
	g_MapStringValues["End"] = evEnd;
}

bool IsValidCommand(std::string& str)
{
	std::map<std::string, StringValue>::iterator it = g_MapStringValues.find(str);
	if (it != g_MapStringValues.end()) return true;
	else return false;
}

#define SSWITCH( STRING )	switch( g_MapStringValues[STRING] )

#endif