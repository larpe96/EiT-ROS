#ifndef PARTSLIST_H
#define PARTSLIST_H
#include <string>
#include <vector>
#include<string>

class PartsList
{
public:
    PartsList();
    void loadPartsList(std::string file);
    void loadPartsList(std::vector<std::string> parts);
    std::vector<std::string> checkMissingParts(std::vector<std::string> parts);
    std::vector<std::string> getPartsList();
    std::vector<std::string> tokenize(std::string s, std::string del = " ");

private:
    std::vector<std::string> parts_list;  // Sorted at loading


};

#endif // PARTSLIST_H
