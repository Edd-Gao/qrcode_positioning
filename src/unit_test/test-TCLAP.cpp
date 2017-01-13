#include <ecl/command_line.hpp>
#include <iostream>
#include <string>

//the ecl has a convenient c++ comand line argument handler ported from TCLAP

int main(int argc, char** argv){

        ecl::CmdLine cmd("this is a test");

        ecl::SwitchArg debug("d","debug","Enable dubugging.",false);
        ecl::ValueArg<std::string> path("f","file","file to read",false,"homer","string");

        cmd.add(debug);
        cmd.add(path);

        cmd.parse(argc, argv);
        std::cout<<"switch:"<<debug.getValue()<<std::endl;
        std::cout<<"path:"<<path.getValue()<<std::endl;
}