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
// * Neither the name of Willow Garage, Inc. nor the names of its
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
 * @file ros_bin.cpp
 *
 * @brief Launch a Python script located in the same location and have the same name as the compiled executable
 *
 * @date March 2011
 */

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <cstdlib>
#include <sstream>
#include <algorithm>
#ifdef _MSC_VER
  #include <windows.h>
#endif

char exe_name[MAX_PATH];
std::stringstream arguments;
std::string python_home, python_script, python_exe;

/*****************************************************************************
** Functions
*****************************************************************************/

void debug() {
    std::cout << std::endl;
    std::cout << "Program Variables:" << std::endl;
    std::cout << "  Python exe: " << python_exe << std::endl;
    std::cout << "  Executable: " << exe_name << std::endl;
    std::cout << "  Python script: " << python_script << std::endl;
    std::cout << "  Arguments: " << arguments.str() << std::endl;
    std::cout << std::endl;
}

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {
    exe_name[0] = '\0';
#ifdef WIN32
    //_splitpath_s(argv[0], NULL, 0, NULL, 0, name, 256, NULL, 0);

    // could use GetModuleHandleW, WCHAR, GetModuleFileNameW, wcout and wstring here instead.
    HMODULE hModule = GetModuleHandle(NULL);
    GetModuleFileName(hModule, exe_name, MAX_PATH);
    python_script = std::string(exe_name);
    python_script.replace(python_script.end()-4, python_script.end(), ""); // replace trailing ".exe" with ""

    python_exe = python_home + std::string("python");
    arguments << python_exe << " " << python_script;
    for ( int i = 1; i < argc; ++i ) {
        // add double quotation marks to handle spaces
        arguments << " \"" << argv[i] << "\"";
    }

    /* TODO: Need some validation checks here! */

    STARTUPINFO startup_info;
    PROCESS_INFORMATION process_info;
    memset(&startup_info, 0, sizeof(startup_info));
    memset(&process_info, 0, sizeof(process_info));

    startup_info.cb = sizeof(startup_info);

    int result =
        CreateProcess(
            NULL, 
            const_cast<char*>(arguments.str().c_str()),
            NULL,
            NULL,
            FALSE,
            0, // CREATE_NEW_CONSOLE,
            NULL,
            NULL,
            &startup_info,
            &process_info
            );
    if ( !result ) {
        unsigned long last_error = GetLastError();
        switch ( last_error ) {
            case ( ERROR_FILE_NOT_FOUND ) : {
                std::cout << "The python executable could not be found - check it is in your PATH" << std::endl;
                debug();
                break;
            }
            default: {
                std::cout << "Process failed with error: " << last_error << std::endl;
                debug();
                break;
            }
        }
        return last_error;
    } else {
        WaitForSingleObject( process_info.hProcess, INFINITE );
        unsigned long exit_code = NO_ERROR;
        GetExitCodeProcess( process_info.hProcess, &exit_code );
        CloseHandle( process_info.hProcess );
        CloseHandle( process_info.hThread );
        return exit_code;
    }
#else
    std::cout << "This is a windows application only." << std::endl;
#endif
    return 0;
}
