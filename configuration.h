#pragma once

#include <map>
#include <string>
using std::string;


class configuration
{
public:
	configuration(string pathname);
	~configuration();

	string lookup(string key);

	string reverse_lookup(string val, bool get_last = false);

	void update(string key, string value);

	void write_configuration();

private:
	std::map< string, string> m_config;

};

extern configuration config;

#define CONFIG "/home/cerulean/cm.config.txt"
