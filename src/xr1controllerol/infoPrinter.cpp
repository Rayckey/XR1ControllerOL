#include "infoPrinter.h"
#include <stdio.h>
#include <iostream>

InfoPrinter::InfoPrinter(const string &appName, const string &version, const string &releaseTime) :
    _appName(appName)
  ,_version(version)
  ,_releaseTime(releaseTime)
{

}

void InfoPrinter::addCustomInfo(const string &cusInfo)
{
    _customInfo = cusInfo;
}

void InfoPrinter::showInfo()
{
    std::cout << _appName.c_str() << " " << _version.c_str() << " ( " << _releaseTime << " )" << std::endl;
    std::cout << "    _____   _   _   _   _   ______    ____     _____ " << std::endl;
    std::cout << "   |_   _| | \\ | | | \\ | | |  ____|  / __ \\   / ____|" << std::endl;
    std::cout << "     | |   |  \\| | |  \\| | | |__    | |  | | | (___  " << std::endl;
    std::cout << "     | |   | . ` | | . ` | |  __|   | |  | |  \\___ \\ " << std::endl;
    std::cout << "    _| |_  | |\\  | | |\\  | | |      | |__| |  ____) |" << std::endl;
    std::cout << "   |_____| |_| \\_| |_| \\_| |_|       \\____/  |_____/ " << std::endl;

    if(_customInfo.length() > 0)
    {
        std::cout <<"-----------------------------------------------------------------" << std::endl;
        cout << _customInfo.c_str() << endl;
        std::cout <<"-----------------------------------------------------------------" << std::endl;
    }
}
