#include "string.hxx"

r2ps::utils::String::String()
{
}

r2ps::utils::String::~String()
{
}

int r2ps::utils::String::parse_int(std::string str)
{
    return std::stoi(str);
}

std::vector<std::string> r2ps::utils::String::split(const std::string &str, char delimiter)
{
    std::vector<std::string> tokens;
    std::stringstream ss(str);
    std::string token;
    while (std::getline(ss, token, delimiter))
    {
        tokens.push_back(token);
    }
    return tokens;
}