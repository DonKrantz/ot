#include <map>
#include <stdio.h>

#include "configuration.h"
#include "utilities.h"
#include <fstream>


namespace {


} // namespace

configuration config(CONFIG); 

configuration::configuration(string pathname)
{
	m_config["user"] = "cerulean";  // print the database with essenmtial defauls in case file not found
	m_config["ip-boot-script-location"] = "/root/fixup_network.sh";
	m_config["reboot-on-change-static-ip"] = "no";
	m_config["webhome"] = "/home/cerulean/web";
	m_config["userhome"] = "/home/cerulean";
	m_config["logger-maxnum"] = "10";
	m_config["logger-maxsize"] = "100";
	m_config["static-ip"] = "192.168.2.19";
	m_config["http-port"] = "80";

	std::fstream f;
	f.std::fstream::open(pathname, std::ios_base::in);

	if (f.fail())
	{
		log_severe("Failed opening configuration file %s", pathname.c_str());
		return;
	}

	while (!f.eof())
	{
		string s;
		std::getline(f, s);

		// eliminate lines starting with hash mark
		if (s[0] == '#')
			continue;

		// eliminate everything including and after the first hash mark #
		size_t p = s.find_first_of("#");
		if( p != string::npos)
			s = s.substr(0, p);;

		// eat up trailing whitespace
		size_t w = s.find_last_not_of(" \t\r");
		if (p != string::npos)
			s = s.substr(0, w + 1);

		// eat up leading whitespace, loop if no non-whitespace
		p = s.find_first_not_of(" \t\r");
		if (p == string::npos)
			continue;
		s = s.substr(p, string::npos);

		m_config[keyvalkey(s, "=")] = keyvalvalue(s, "=");
	}

	f.close();
}

configuration::~configuration()
{
}

string configuration::lookup(string key)
{
	try
	{
		return m_config.at(key);
	}
	catch(...)
	{
		return "";
	}
}

string configuration::reverse_lookup(string val, bool get_last)
{
	if (get_last)
	{
		auto ptr = m_config.end();
		while (true)
		{
			if (ptr == m_config.begin())
				return "";

			ptr--;

			if ((*ptr).second == val)
				return (*ptr).first;
		}
	}
	else
	{
		auto ptr = m_config.begin();
		while (true)
		{
			if (ptr == m_config.end())
				return "";

			if ((*ptr).second == val)
				return (*ptr).first;

			ptr++;
		}
	}
}


void configuration::update(string key, string value)
{
	m_config[key] = value;
}

void configuration::write_configuration()
{
	std::fstream f;
	f.std::fstream::open(CONFIG ".temp", std::ios_base::out);

	if (f.fail())
	{
		log_severe("Failed opening configuration file temp %s", CONFIG ".temp");
		return;
	}

	f.std::fstream::write("# Automatically Generated\n", strlen("# Automatically Generated\n"));

	auto index = m_config.begin();
	while (index != m_config.end())
	{
		string tuple = index->first + "=" + index->second + "\n";
		f.std::fstream::write(tuple.c_str(), tuple.length());
		index++;
	}

	f.std::fstream::close();

	string cmd = "mv -f " + (string)(CONFIG ".temp") + " " + CONFIG;
	system(cmd.c_str());

	sudo_mode();
	cmd = "chown " + config.lookup("user") + ":" + config.lookup("user") + " " CONFIG;
	system(cmd.c_str());
	user_mode();

	log_event("Configuration file written");
}