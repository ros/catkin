/**
 * @file /src/ros_bin.cpp
 *
 * @brief Self describing executable for launching python scripts in /bin
 *
 * @date March 2011
 **/

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
		// need the quotes to make sure spaces dont muck things up
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
			const_cast<char*>(arguments.str().c_str()), // bloody windoze
			NULL,
			NULL,
			FALSE,
			0, // CREATE_NEW_CONSOLE,
			NULL,
			NULL,
			&startup_info,
			&process_info
			);
	if ( result == 0 ) {
		switch ( result ) {
			case ( ERROR_FILE_NOT_FOUND ) : {
				std::cout << "The python executable could not be found - check it is in your PATH" << std::endl;
				debug();
				break;
			}
			default: {
				std::cout << "Process failed with error: " << GetLastError() << std::endl;
				debug();
				break;
			}
		}
	} else {
		WaitForSingleObject( process_info.hProcess, INFINITE );
	    CloseHandle( process_info.hProcess );
	    CloseHandle( process_info.hThread );
	}
#else
	std::cout << "This is a windows application only." << std::endl;
#endif
	return 0;
}




