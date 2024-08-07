#include "r2ps_utils/process.hpp"

r2ps::utils::Process::Process()
{
}

r2ps::utils::Process::~Process()
{
}

std::string r2ps::utils::Process::execute_command(std::string command)
{
    const char *_command = command.c_str();

    try
    {
        const int &command_result = system(_command);

        if (command_result == 0)
        {
            FILE *pipe = popen(_command, "r");
            if (!pipe)
            {
                printf("Failed to run command: %s", command);
                return "";
            }

            char buffer[128];
            std::string result = "";

            while (fgets(buffer, sizeof(buffer), pipe) != nullptr)
            {
                result += buffer;
            }

            const int &exit_status = pclose(pipe);

            return result;
        }
        else
        {
            printf("Failed to run command: %s", command);
            return "";
        }
    }
    catch (const std::exception &e)
    {
        printf("Failed to run command: %s", command);
        return "";
    }
}