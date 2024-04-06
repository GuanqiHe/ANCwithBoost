#pragma once
#include <iostream>
#include <vector>
#include <string>
#include <fstream>

#include "msgpack.hpp"

struct data_logger
{
	std::vector<std::string> name;
	std::vector<std::vector<double>> data;
	std::string file_path;
	std::vector<std::vector<double>>::iterator it;

	void log(std::initializer_list<double> &&element)
	{
		*it = (std::move(element));
		it++;
	}

	void init(std::vector<std::string> name_list, int sample_len, std::string path)
	{
		name = name_list;
		data = std::vector<std::vector<double>>(sample_len, std::vector<double>(name.size(), 0));
		file_path = path;

		it = data.begin();
	}

	void write()
	{
		std::ofstream stream(file_path, std::ios::binary);
		msgpack::packer<std::ofstream> packer(stream);
		packer.pack(name);
		packer.pack(data);
		stream.close();
	}
};
