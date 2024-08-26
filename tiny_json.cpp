
#include <string>
#include <stdlib.h>

#include "tiny_json.h"

using std::string;


// gets the value from a JSON string given the key (without quotes around the key).
string next_element(string& json, int skips)
{
	int p = skips;

	while (isspace(json[p]))
		p++;

	int firstchar = p;	// start of string

	int brackets = 0;
	int braces = 0;

	while (true)
	{
		if (p >= (int)json.length())
		{
			return "";
		}

		if ((json[p] == ',') || (json[p] == ']'))
		{
			if ((braces == 0) && (brackets == 0))
			{
				string result = json.substr(firstchar, p - 1);
				json = json.substr(p + 1, string::npos);
				return result;
			}
		}

		if (json[p] == '[')
			brackets++;
		else if (json[p] == ']')
			brackets--;
		else if (json[p] == '{')
			braces++;
		else if (json[p] == '}')
			braces--;
		else if (json[p] == '"')
		{
			p++;
			while (json[p] != '"')
				p++;
		}

		p++;
	}
}


//==================================================================================
//==================================================================================
//==================================================================================
// gets the value from a JSON string given the key (without quotes around the key).
// For curly brace values returns the contents including outer curly braces and
//    contents of nested curly braces
// For quoted strings returnd the string with quotes stripped off
// For square brackets returns contents with square brackets intact
// For everything else (numbers) returns the value
//
// limitations:
// doesn't handle square brackets or escaped quotes or curly braces in strings
// doesn't handle nest square brackets 
//
// Gets the first instance of the pattern even if in braces, but by getting the
// outer fields you can target specific fields by getting the enclosing braces:
//   string u = value_of("quaternion", value_of("message",jstring));
//
string value_of(string pattern, string target)
{
	try
	{
		string sregex = "\"" + pattern + "\"";

		size_t pos = target.find(sregex);

		if (pos == string::npos)
			return "";

		pos += sregex.length();

		while (std::isspace(target[pos]))
			pos++;
		while (target[pos] == ':')
			pos++;
		while (std::isspace(target[pos]))
			pos++;

		// here pos should point at the first non-space character after the colon.
		size_t past = pos + 1;
		if (target[pos] == '{')
		{
			past = pos + 1;
			int braces = 1;

			while (true)
			{
				if (target[past] == '}')
					braces--;
				if (target[past] == '{')
					braces++;
				past++;
				if (braces == 0)
					break;
			}
		}
		else if (target[pos] == '[')
		{
			past = pos + 1;
			int braces = 1;

			while (true)
			{
				if (target[past] == ']')
					braces--;
				if (target[past] == '[')
					braces++;
				past++;
				if (braces == 0)
					break;
			}
		}
		else if (target[pos] == '"')
		{
			past = pos + 1;
			while (target[past] != '"')
				past++;
			pos++;  // strip the quote
		}
		else // a number probably
		{
			past = pos;
			bool is_delim = false;

			while (true)
			{
				char c = target[past];
				switch (c)
				{
				case ' ':
				case '{':
				case '}':
				case '[':
				case ']':
				case ',':
				case '\n':
				case '\r':
					is_delim = true;
					break;
				default:
					is_delim = false;
					break;
				}

				if (is_delim)
					break;
				past++;
			}
		}

		string result = target.substr(pos, past - pos);
		return(result);
	}
	catch (...)
	{
		return "";
	}
}

