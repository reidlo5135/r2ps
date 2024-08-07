#ifndef PROCESS__HPP
#define PROCESS__HPP

#include <memory>
#include <sstream>
#include <vector>

namespace r2ps
{
    namespace utils
    {
        class Process
        {
        public:
            explicit Process();
            virtual ~Process();
            std::string execute_command(std::string command);
        public:
            using SharedPtr = std::shared_ptr<Process>;
        };
    }
}

#endif // !PROCESS__HPP