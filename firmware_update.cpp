
#include <stdio.h>
#include <sys/socket.h>
#include <unistd.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/types.h>
#include <fcntl.h>
#include <linux/usbdevice_fs.h>
#include <sys/sendfile.h>
#include <sys/stat.h>
#include <errno.h>
#include <map>

#include <stdlib.h>
#include <algorithm>
#include <vector>
#include <signal.h>
#include <termio.h>
#include <sys/prctl.h>

#include <malloc.h>

#include "webserver.h"
#include "utilities.h"
#include "configuration.h"

#include <curl/curl.h>

#include "firmware_update.h"
#include "tiny_json.h"

#include <list>


namespace {

	const string RepoPrefix = "https://api.github.com/repos/CeruleanSonar/AutoFirmware/contents";
	const string RepoRawPrefix = "https://raw.githubusercontent.com/CeruleanSonar/AutoFirmware/master/";
	const string RepoName = RepoPrefix + "contents";

//	char curl_error_string[CURL_ERROR_SIZE + 1];

#if false
	//==================================================================================
	// CURL helper
	size_t read_function(char* bufptr, size_t size, size_t nitems, void* userp)
	{
	}
#endif

#if false
	//==================================================================================
	// CURL helper
	size_t write_data_post(void* buffer, size_t size, size_t nmemb, void* userp)
	{
		static char last[132] = { 0 };

		strncpy(last, (char*)buffer, sizeof(last) - 1);

		return size * nmemb;
	}
#endif

	//==================================================================================
	// CURL helper
	struct MemoryStruct 
	{
		char* memory;
		size_t size;
	};

	//==================================================================================
	// CURL helper
	size_t write_data_get(void* buffer, size_t size, size_t nmemb, void* userp)
	{
		size_t realsize = size * nmemb;
		struct MemoryStruct* mem = (struct MemoryStruct*)userp;

		char* ptr = (char *)realloc(mem->memory, mem->size + realsize + 1);
		if (ptr == NULL) 
		{
			/* out of memory! */
			printf("not enough memory (realloc returned NULL)\n");
			return 0;
		}

		mem->memory = ptr;
		memcpy(&(mem->memory[mem->size]), buffer, realsize);
		mem->size += realsize;
		mem->memory[mem->size] = 0;

		return realsize;
	}

	//==================================================================================
	bool list_contains(string pattern, list<string>alist)
	{
		for (string &s : alist)
			if (s == pattern)
				return true;
		return false;
	}

	//==================================================================================
	struct gitter
	{
		string name;
		string dl_path;
		string size;
	};


	//==================================================================================
	void update_category(int fd, string pattern, string remote, string local, bool must_be_elf)
	{
      string j;

      CURL* curl;
      CURLcode res;

		list<gitter> git_side;

		struct MemoryStruct response = { NULL, 0 };
		response.memory = (char *)malloc(1);

      struct curl_slist* headers = NULL;

      headers = curl_slist_append(headers, "Accept: application/json");
      headers = curl_slist_append(headers, "Content-Type: application/json");

      curl = curl_easy_init();
      if (curl) 
      {
         curl_easy_setopt(curl, CURLOPT_URL, remote.c_str());
         curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_data_get);
         curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
			curl_easy_setopt(curl, CURLOPT_USERAGENT, "Cerulean Smartswitch/1.0"); 
			res = curl_easy_perform(curl);
         curl_easy_cleanup(curl);
      }

		if (response.memory[0] == '[')
		{
			string json = response.memory + 1;
			string element = "";
			while (true)
			{
				element = next_element(json);
				if (element == "")
					break;

				string name = value_of("name", element);
				string dl_path = value_of("download_url", element);
				string size = value_of("size", element);

				if (contains(pattern, name))
					if( must_be_elf)
						if (contains(".elf", name))
							git_side.emplace_back((gitter){ name, dl_path, size });


			}

			list<string> dirlist;
			GetFilesInDirectory(dirlist, local, "", true);

			for (gitter& s : git_side)
			{
				if (list_contains(s.name, dirlist))
				{
					wprintf(fd, "<p>aleady have, skipping %s</p>\r\n", s.name.c_str());
				}
				else
				{
					wprintf(fd, "<p>downloading %s\r\n", s.name.c_str());

					string url = s.dl_path;

					string outfilename = local + "/download_temp";
					response = { (char *)malloc(1), 0 };

					curl = curl_easy_init();

					if (curl) 
					{
						curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
						curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_data_get);
						curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
						curl_easy_setopt(curl, CURLOPT_USERAGENT, "Cerulean Smartswitch/1.0");
						res = curl_easy_perform(curl);
						curl_easy_cleanup(curl);

						if (res == CURLE_OK)
						{
							sudo_mode();
							int ffd = open(outfilename.c_str(), O_CREAT | O_WRONLY);
							user_mode();

							if (ffd < 0)
							{
								log_warning("opening local file '%s' : %s", outfilename.c_str(), strerror(errno));
								wprintf(fd, "<b style='color:red'> - FAILED</b></p>");
								continue;
							}

							size_t size = response.size;
							while (size != 0)
							{
								int n = (int)write(ffd, response.memory + response.size - size, size);
								if (n < 0)
								{
									log_warning("writing local file '%s' : %s", outfilename.c_str(), strerror(errno));
									close(ffd);
									wprintf(fd, "<b style='color:red'> - FAILED</b></p>");
									continue;
								}

								size -= n;
							}

							wprintf(fd, " - completed</p>");

							sudo_mode();
							string cmd = (string)"chmod +wr '" + outfilename + "'";
							system((cmd).c_str());

							cmd = (string)"chown " + config.lookup("user") + " '" + outfilename + "'";
							system((cmd).c_str());

							cmd = (string)"chgrp " + config.lookup("user") + " '" + outfilename + "'";
							system((cmd).c_str());

							cmd = (string)"mv '" + outfilename + "' '" + local + "/" + s.name + "'";
							system((cmd).c_str());

							user_mode();

							close(ffd);
						}
						else
						{
							log_warning("downloading web file '%s' : %s", s.name.c_str(), strerror(errno));
							wprintf(fd, " - failed</p>");
						}
					}
				}
			}
		}
		else
		{
			return;
		}

	}

} // namespace


//==================================================================================
void update_firmware(int fd, string path, string options)
{
	if (contains("tracker650-files=on", options))
	{
		update_category(fd, "Tracker-650", RepoPrefix, config.lookup("userhome") + "/firmware", true);
	}

	if (contains("rovl-files=on", options))
	{
		update_category(fd, "ROVL-", RepoPrefix, config.lookup("userhome") + "/firmware", true);
	}

	if (contains("dvl75-files=on", options))
	{
		update_category(fd, "DVL-", RepoPrefix, config.lookup("userhome") + "/firmware", true);
	}

	if (contains("smartswitch-files=on", options))
	{
		update_category(fd, "smartswitch", RepoPrefix, config.lookup("userhome") + "/firmware", false);
	}
}
