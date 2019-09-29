#ifndef _INFO_PRINTER_H___
#define _INFO_PRINTER_H___
#include <string>
using namespace std;

class InfoPrinter
{
public:
    InfoPrinter(const string & appName,const string & version,const string & releaseTime);
    void addCustomInfo(const string &cusInfo);
    void showInfo();
private:
    string _appName;
    string _version;
    string _customInfo;
    string _releaseTime;
};

#endif //_INFO_PRINTER_H__
