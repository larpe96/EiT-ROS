#include "parts_list_pkg/partslist.h"
#include <algorithm>
#include <string>
#include <fstream>
#include <bits/stdc++.h>
//#include <functional>

PartsList::PartsList()
{

}

void PartsList::loadPartsList(std::string file)
{
    this->parts_list.clear();

    std::vector<std::string> data;
    std::string line;

    std::ifstream parts_file(file);
    if (parts_file.is_open())
    {
        line.erase(std::remove(line.begin(), line.end(), '\n'), line.end());

        for(line; std::getline(parts_file, line);/**/)
        {
            line.erase(std::remove(line.begin(), line.end(), '\n'), line.end());
            data.push_back(line);
        }

        parts_file.close();
    }

    // Convert from file format to format of parts_list
    for(int i = 0; i < data.size(); i++)
    {
        std::string tmp_string = data[i];
        //std::cout << tmp_string << std::endl;
        std::vector<std::string> split_tmp_string = tokenize(tmp_string);
       // std::cout << split_tmp_string[0] << "-" << split_tmp_string.back() << std::endl;
        std::string tmp_amount_str = split_tmp_string.back();
        //std::cout << "tmp_amount_str: " <<tmp_amount_str << std::endl;
        int tmp_amount = std::stoi(tmp_amount_str);
        std::string tmp_part = split_tmp_string[0];

        for(int j = 0; j < tmp_amount; j++)
        {
            this->parts_list.push_back(tmp_part);
        }
    }
    std::sort(this->parts_list.begin(), this->parts_list.end());
}

void PartsList::loadPartsList(std::vector<std::string> parts)
{
    this->parts_list = parts;
    std::sort(this->parts_list.begin(), this->parts_list.end());
}

std::vector<std::string> PartsList::checkMissingParts(std::vector<std::string> parts)
{
    std::sort(parts.begin(), parts.end());

    int size_diff = std::max(parts.size(), parts_list.size());
    std::vector<std::string> diff(size_diff);

    std::set_difference(this->parts_list.begin(), this->parts_list.end(), parts.begin(), parts.end(), diff.begin());

    // Remove empty spots in diff vector
    std::vector<std::string> missing_parts;

    for(int i = 0; i < diff.size(); i++)
    {
        if(diff[i] != "")
        {
            missing_parts.push_back(diff[i]);
        }
    }

    return missing_parts;
}

std::vector<std::string> PartsList::getPartsList()
{
    return this->parts_list;
}

std::vector<std::string> PartsList::tokenize(std::string s, std::string del)
{
    std::vector<std::string> output;

    int start = 0;
    int end = s.find(del);
    while (end != -1)
    {
        output.push_back(s.substr(start, end - start));
        start = end + del.size();
        end = s.find(del, start);
    }

    output.push_back(s.substr(start, end - start));

    return output;
}
