//
// Software License Agreement (BSD License)
//
// Copyright (c) 2011, Yujin Robot
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of Yujin Robot nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

/*
 * @file python_win32_wrapper.cpp
 *
 * @brief Launch a Python script located in the same location and have the same name as the compiled executable
 *
 * @date March 2011
 */

#ifdef _WIN32

#include <iostream>
#include <windows.h>

int main(int argc, char* argv[]) try
{
    const auto GetCurrentModuleName = []() -> std::string
    {
        // initialize buffer string with at least one character
        std::string moduleName = " ";
        while (true)
        {
            // retrieves the path of the executable file of the current process
            auto result = ::GetModuleFileName(nullptr, &moduleName[0], static_cast<DWORD>(moduleName.size()));
            if (!result)
            {
                throw ::GetLastError();
            }
            else if (result == moduleName.size() && ::GetLastError() == ERROR_INSUFFICIENT_BUFFER)
            {
                // buffer not large enough
                moduleName.resize(moduleName.size() * 2);
                continue;
            }
            moduleName.resize(result);
            break;
        }
        return moduleName;
    };

    const auto FindPythonScript = [](const std::string& exeName) -> std::string
    {
        // this could become more fail-proof and readable with https://en.cppreference.com/w/cpp/filesystem from c++20
        const std::string exeExtension = ".exe";
        if (exeName.size() <= exeExtension.size() || exeName.substr(exeName.size() - exeExtension.size()) != exeExtension)
        {
            throw L"Invalid name.";
        }

        // use quoted string to ensure the correct path is used
        return "\"" + exeName.substr(0, exeName.size() - exeExtension.size()) + "\"";
    };

    const auto GetPythonExecutable = []() -> std::string
    {
        std::string pythonExecutable = "python";

        // use quoted string to indicate where the file name ends and the arguments begin
        return "\"" + pythonExecutable + "\"";
    };

    const auto pythonExecutable = GetPythonExecutable();
    const auto pythonScript = FindPythonScript(GetCurrentModuleName());
    std::string command = pythonExecutable + " " + pythonScript;
    for (auto i = 1; i < argc; ++i)
    {
        command += " ";
        // use quoted strings to handle spaces within each argument
        command += " \"" + std::string(argv[i]) + "\"";
    }

#if defined(DEBUG)
    for (auto i = 0; i < argc; ++i)
    {
        fprintf(stderr, "[DEBUG] %d:\t%s\n", i, argv[i]);
    }

    fprintf(stderr, "[DEBUG] script: %s\n", pythonScript.c_str());
    fprintf(stderr, "[DEBUG] command: %s\n", command.c_str());
#endif

    STARTUPINFO startup_info;
    PROCESS_INFORMATION process_info;
    ::memset(&startup_info, 0, sizeof(startup_info));
    ::memset(&process_info, 0, sizeof(process_info));
    startup_info.cb = sizeof(startup_info);

    auto result = ::CreateProcess(
        nullptr,                // program to execute (nullptr = execute command line)
        &command[0],            // command line to execute
        nullptr,                // process security attributes
        nullptr,                // thread security attributes
        false,                  // determines if handles from parent process are inherited
        0,                      // no creation flags
        nullptr,                // enviornment (nullptr = use parent's)
        nullptr,                // current directory (nullptr = use parent's)
        &startup_info,          // startup info
        &process_info           // process info
    );
    if (!result)
    {
        const auto error = ::GetLastError();
        switch (error)
        {
        case ERROR_FILE_NOT_FOUND:
        {
            fprintf(stderr, "Error! Python executable cannot be found, is it added to PATH?\n");
            break;
        }
        default:
            fprintf(stderr, "Error! CreateProcess failed with error code: %ld\n", error);
            break;
        }
        throw error;
    }
    ::WaitForSingleObject(process_info.hProcess, INFINITE);
    unsigned long exitCode = NO_ERROR;
    ::GetExitCodeProcess(process_info.hProcess, &exitCode);
    ::CloseHandle(process_info.hProcess);
    ::CloseHandle(process_info.hThread);

#if defined(DEBUG)
    fprintf(stderr, "[DEBUG] process exist code: %ld\n", exitCode);
#endif

    return exitCode;
}
catch (...)
{
    fprintf(stderr, "Failed to execute the Python script...\n");
    return 1;
}

#else

#error This wrapper should only be created on Windows.

int main()
{
    // deliberately add syntax error when the file is not supposed to get compiled
    return 0
}

#endif
