#ifndef STRING__HXX
#define STRING__HXX

#include <memory>
#include <string>
#include <sstream>
#include <vector>

namespace r2ps
{
    namespace utils
    {
        class String
        {
        public:
            explicit String();
            virtual ~String();
            using SharedPtr = std::shared_ptr<String>;
        public:
            int parse_int(std::string str);
            std::vector<std::string> split(const std::string &str, char delimiter);
        };
    }
}

#endif // !STRING__HXX