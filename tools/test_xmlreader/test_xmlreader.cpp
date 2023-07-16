#include <iostream>
#include "xmlreader.h"

using namespace APP_NAMESPACE;

int main(int argc, char *argv[])
{
    //==========================================================================
    std::cout << "\n" + std::string(80, '=') + "\nread xml from file\n" + std::string(80, '=') + "\n\n";

    std::cout << "\n<-- valid file name -->\n\n";

    try {
        XmlReader xml("parameter.xml");
        std::cout << "Succeeded to read the xml file: " << xml.getXmlFilePathName() << "\n";
        std::cout << xml.readXml() << "\n";
    }
    catch (const std::exception &e) {
        std::cout << "Failed to read the xml file: " << e.what() << "\n";
        return 1;
    }
    
    std::cout << "\n<-- invalid file name -->\n\n";

    try {
        XmlReader xml("parameter0.xml");
        std::cout << "Succeeded to read the xml file: " << xml.getXmlFilePathName() << "\n";
        std::cout << xml.readXml() << "\n";
    }
    catch (const std::exception &e) {
        std::cout << "Failed to read the xml file: " << e.what() << "\n";
    }

    //==========================================================================
    std::cout << "\n" + std::string(80, '=') + "\nread xml from string\n" + std::string(80, '=') + "\n\n";

    std::cout << "\n<-- valid xml string -->\n\n";

    try {
        XmlReader xml("parameter.xml");
        XmlReader xml2;
        xml2.writeXml(xml.readXml());
        std::cout << xml2.readXml() << "\n";
    }
    catch (const std::exception &e) {
        std::cout << "Failed to write the xml string: " << e.what() << "\n";
        return 1;
    }

    std::cout << "\n<-- invalid xml string -->\n\n";

    try {
        XmlReader xml;
        xml.writeXml("hello world");
        std::cout << xml.readXml() << "\n";
    }
    catch (const std::exception &e) {
        std::cout << "Failed to write the xml string: " << e.what() << "\n";
    }

    //==========================================================================
    std::cout << "\n" + std::string(80, '=') + "\ntest getData and putData\n" + std::string(80, '=') + "\n\n";

    try {
        XmlReader xml("parameter.xml", '.');

        std::cout << "<-- get data -->\n";

        std::string system_name = xml.getData("param.system.info.name");
        std::string system_name_attr = xml.getData("param,system,info,<xmlattr>,name", ',');
        int dim_x = atoi(xml.getData("param;system;dim;<xmlattr>;x", ';'));
        double Kp = atof(xml.getData("param control PID gains <xmlattr> Kp", ' '));
        std::cout << "system_name = " << system_name << "\n";
        std::cout << "system_name_attr = " << system_name_attr << "\n";
        std::cout << "dim_x = " << dim_x << "\n";
        std::cout << "Kp = " << Kp << "\n";
        
        std::cout << "<-- put data -->\n";

        xml.putData("param.system.info.name", "my_system");
        xml.putData("param,system,info,<xmlattr>,name", "my_system (attr)", ',');
        xml.putData("param;system;dim;<xmlattr>;x", "12", ';');
        xml.putData("param control PID gains <xmlattr> Kp", "110.5", ' ');
        
        system_name = xml.getData("param.system.info.name");
        system_name_attr = xml.getData("param.system.info.<xmlattr>.name");
        dim_x = atoi(xml.getData("param.system.dim.<xmlattr>.x"));
        Kp = atof(xml.getData("param.control.PID.gains.<xmlattr>.Kp"));
        std::cout << "system_name = " << system_name << "\n";
        std::cout << "system_name_attr = " << system_name_attr << "\n";
        std::cout << "dim_x = " << dim_x << "\n";
        std::cout << "Kp = " << Kp << "\n";
    }
    catch (const std::exception &e) {
        std::cout << "Failed to get and put data: " << e.what() << "\n";
        return 1;
    }

    std::cout << "\n<-- wrong path for getData() -->\n\n";

    try {
        XmlReader xml("parameter.xml");
        std::string system_name = xml.getData("param.system.info.name2");
        std::cout << "system_name = " << system_name << "\n";
    }
    catch (const std::exception &e) {
        std::cout << "Failed to get data: " << e.what() << "\n";
    }

    std::cout << "\n<-- new path for putData() -->\n\n";

    try {
        XmlReader xml("parameter.xml");
        xml.putData("param.system.info.name2", "my_system");
        xml.putData("param.system.info.<xmlattr>.name2", "my_system (attr)");
        std::cout << xml.readXml() << "\n";
    }
    catch (const std::exception &e) {
        std::cout << "Failed to put data: " << e.what() << "\n";
    }

    //==========================================================================
    std::cout << "\n" + std::string(80, '=') + "\ncopy xml file\n" + std::string(80, '=') + "\n\n";

    try {
        XmlReader xml("parameter.xml");
        xml.putData("param.system.info.name", "my_system");
        xml.putData("param.system.info.<xmlattr>.name", "my_system (attr)");
        xml.saveFile("parameter2.xml");
        XmlReader xml2("parameter2.xml");
        std::cout << xml2.readXml() << "\n";
        std::remove("parameter2.xml");
    }
    catch (const std::exception &e) {
        std::cout << "Failed to copy the xml file: " << e.what() << "\n";
        return 1;
    }

    //==========================================================================
    std::cout << "\n" + std::string(80, '=') + "\ncreate xml file\n" + std::string(80, '=') + "\n\n";

    try {
        XmlReader xml;
        xml.putData("param.test.<xmlattr>.name", "Hello World!");
        xml.putData("param.test.name", "Goodbye World!");
        xml.saveFile("parameter2.xml");
        XmlReader xml2;
        xml2.loadFile("parameter2.xml");
        std::cout << xml2.readXml() << "\n";
        std::remove("parameter2.xml");
    }
    catch (const std::exception &e) {
        std::cout << "Failed to create the xml file: " << e.what() << "\n";
        return 1;
    }

    return 0;
}